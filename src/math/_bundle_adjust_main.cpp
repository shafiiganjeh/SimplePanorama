
#include "_bundle_adjust_main.h"


namespace bundm {

    adjuster::adjuster(const std::vector<util::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,float lmbd,const imgm::pan_img_transform &Tr,int threads){

        thr = threads;
        par_er = std::make_shared<class bund::E_func>(kp,match,*Tr.adj);
        par_img = std::make_shared<class bund::parameters>(kp,match,Tr,thr);
        iter.lambda = lmbd;
        iter.hom = par_img -> ret_hmat();

    }


    void adjuster::calc_uf(int thread,const std::vector<bund::A_vec>& avec,std::vector<Eigen::MatrixXd>& thread_accumulators){

        for(const int & i : threads_vector[thread]){

            const bund::A_vec& a = avec[i];

            thread_accumulators[thread].block<6, 6>(a.idx_i * 6, a.idx_i * 6).noalias() += a.Ai.transpose() * a.Ai;
            thread_accumulators[thread].block<6, 6>(a.idx_i * 6, a.idx_j * 6).noalias() += a.Ai.transpose() * a.Aj;
            thread_accumulators[thread].block<6, 6>(a.idx_j * 6, a.idx_i * 6).noalias() += a.Aj.transpose() * a.Ai;
            thread_accumulators[thread].block<6, 6>(a.idx_j * 6, a.idx_j * 6).noalias() += a.Aj.transpose() * a.Aj;
        }

    }

    Eigen::MatrixXd adjuster::ret_uf(const std::vector<bund::A_vec>& avec, int total_cols) {

        std::vector<Eigen::MatrixXd> thread_accumulators(thr);
        std::vector<std::thread> thread_objects;

        for (auto& mat : thread_accumulators) {
            mat = Eigen::MatrixXd::Zero(total_cols, total_cols);
        }

        for(int k = 0;k < threads_vector.size();k++){

            std::thread tobj(&adjuster::calc_uf,this,k,std::ref(avec),std::ref(thread_accumulators));
            thread_objects.push_back(std::move(tobj));

        }

        for (auto& t : thread_objects) {
            t.join();
        }

        Eigen::MatrixXd U = Eigen::MatrixXd::Zero(total_cols, total_cols);
        for (auto& accum : thread_accumulators) {
            U.noalias() += accum;
        }

        return U;
    }


    std::vector<Eigen::MatrixXd> adjuster::sum_transpose(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::MatrixXd> &avec){

        std::vector<Eigen::MatrixXd> W;

        for(int i = 0;i < avec.size();i++){

            W.push_back(avec[i].transpose() * bvec[i]);

        }

        return W;
    }


    void adjuster::sum_transposeAB_calc(int thread,const std::vector<bund::A_vec>& avec,const std::vector<Eigen::MatrixXd>& bvec,std::vector<Eigen::MatrixXd> &W){

        for(const int & i : threads_vector[thread]){

            const bund::A_vec& a = avec[i];
            const int total_rows = 6 * a.size;
            Eigen::MatrixXd Wi = Eigen::MatrixXd::Zero(total_rows, 2);

            Wi.middleRows(a.idx_i * 6, 6) = a.Ai.transpose() * bvec[i];
            Wi.middleRows(a.idx_j * 6, 6) = a.Aj.transpose() * bvec[i];

            W[i] = std::move(Wi);

        }

    }


    std::vector<Eigen::MatrixXd> adjuster::sum_transposeAB(const std::vector<Eigen::MatrixXd>& bvec,const std::vector<bund::A_vec>& avec){

        std::vector<Eigen::MatrixXd> W(avec.size());
        std::vector<std::thread> thread_objects;

        for(int k = 0;k < threads_vector.size();k++){

            std::thread tobj(&adjuster::sum_transposeAB_calc,this,k,std::ref(avec),std::ref(bvec),std::ref(W));
            thread_objects.push_back(std::move(tobj));

        }

        for (auto& t : thread_objects) {
            t.join();
        }

        return W;
    }


    void adjuster::ret_Ea_calc(int thread,const std::vector<bund::A_vec>& avec,const std::vector<Eigen::VectorXd>& e_vec,std::vector<Eigen::VectorXd> &thread_Ev){

        auto& local_Ev = thread_Ev[thread];
        for(const int & i : threads_vector[thread]){

            const bund::A_vec& a = avec[i];
            const Eigen::VectorXd& e = e_vec[i];

            local_Ev.segment<6>(a.idx_i * 6).noalias() += a.Ai.transpose() * e;
            local_Ev.segment<6>(a.idx_j * 6).noalias() += a.Aj.transpose() * e;

        }

    }


    Eigen::VectorXd adjuster::ret_Ea(const std::vector<bund::A_vec>& avec,const std::vector<Eigen::VectorXd>& e_vec){

        const int total_cols = avec[0].size * 6;
        std::vector<Eigen::VectorXd> thread_Ev(thr,Eigen::VectorXd::Zero(total_cols));
        std::vector<std::thread> thread_objects;
        Eigen::VectorXd Ev = Eigen::VectorXd::Zero(total_cols);

        for(int k = 0;k < threads_vector.size();k++){

            std::thread tobj(&adjuster::ret_Ea_calc,this,k,std::ref(avec),std::ref(e_vec),std::ref(thread_Ev));
            thread_objects.push_back(std::move(tobj));

        }

        for (auto& t : thread_objects) {
            t.join();
        }

        for (auto& vec : thread_Ev) {
            Ev.noalias() += vec;
        }

        return Ev;
    }


    std::vector<Eigen::VectorXd> adjuster::ret_Eb(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::VectorXd> &e_vec){

        std::vector<Eigen::VectorXd> Ev;

        for(int i = 0;i < bvec.size();i++){

            Ev.push_back(bvec[i].transpose() * e_vec[i]);

        }

        return Ev;
    }


    std::vector<bund::A_vec> adjuster::get_iter_par() {
/*

        std::vector<bund::A_vec> avec = par_img->ret_A_i();
        int size_tot = avec[0].size * 6;

        std::vector<Eigen::MatrixXd> bvec = par_img->ret_B_i();

        std::vector<Eigen::VectorXd> ms = par_img->ret_measurements_saved();

        iter.e_vec = par_er->error(ms);
        iter.u_vecf = ret_uf(avec, size_tot);
        iter.v_vec = sum_transpose(bvec, bvec);
        iter.w_vec = sum_transposeAB(bvec, avec);

        iter.eA_vec = ret_Ea(avec, iter.e_vec);
        iter.eB_vec = ret_Eb(bvec, iter.e_vec);

*/

        auto avec_fut = std::async(std::launch::async, [this]() {
            return par_img->ret_A_i();
        });

        auto bvec_fut = std::async(std::launch::async, [this]() {
            return par_img->ret_B_i();
        });

        auto ms_fut = std::async(std::launch::async, [this]() {
            return par_img->ret_measurements();
        });

        std::vector<bund::A_vec> avec = avec_fut.get();
        int size_tot = avec[0].size * 6;
        std::vector<Eigen::MatrixXd> bvec = bvec_fut.get();
        std::vector<Eigen::VectorXd> ms = ms_fut.get();

        auto uf_fut = std::async(std::launch::async, [this, &avec, size_tot]() {
            return ret_uf(avec, size_tot);
        });

        auto vvec_fut = std::async(std::launch::async, [this, &bvec]() {
            return sum_transpose(bvec, bvec);
        });

        auto wvec_fut = std::async(std::launch::async, [this, &bvec, &avec]() {
            return sum_transposeAB(bvec, avec);
        });

        auto evec_fut = std::async(std::launch::async, [this, &ms]() {
            return par_er->error(ms);
        });

        iter.e_vec = evec_fut.get();

        auto eA_fut = std::async(std::launch::async, [this, &avec]() {
            return ret_Ea(avec, iter.e_vec);
        });

        auto eB_fut = std::async(std::launch::async, [this, &bvec]() {
            return ret_Eb(bvec, iter.e_vec);
        });

        iter.u_vecf = uf_fut.get();
        iter.v_vec = vvec_fut.get();
        iter.w_vec = wvec_fut.get();
        iter.eA_vec = eA_fut.get();
        iter.eB_vec = eB_fut.get();

        return avec;
    }


    void adjuster::augment_calc(int thread,double aug_lamda,const std::vector<bund::A_vec> &avec,struct inter_par& iter){

        Eigen::Matrix2d temp;
        for(const int & p : threads_vector[thread]){

                const bund::A_vec& a = avec[p];
                iter.v_vec_augmented[p].diagonal() =iter.v_vec_augmented[p].diagonal()*aug_lamda;

                double denom = 1.0/(iter.v_vec_augmented[p](0,0)*iter.v_vec_augmented[p](1,1) - iter.v_vec_augmented[p](0,1)*iter.v_vec_augmented[p](1,0));
                temp(0,0) = iter.v_vec_augmented[p](1,1);
                temp(0,1) = -iter.v_vec_augmented[p](0,1);
                temp(1,0) = -iter.v_vec_augmented[p](1,0);
                temp(1,1) = iter.v_vec_augmented[p](0,0);
                iter.v_vec_augmented[p] = denom*temp;

                iter.Y_vec[p].block<6, 2>(a.idx_i * 6,0) = iter.w_vec[p].block<6, 2>(a.idx_i * 6,0) * iter.v_vec_augmented[p];
                iter.Y_vec[p].block<6, 2>(a.idx_j * 6,0) = iter.w_vec[p].block<6, 2>(a.idx_j * 6,0) * iter.v_vec_augmented[p];

        }

    }


    void adjuster::augment(const std::vector<bund::A_vec> &avec){

        iter.u_vecf_augmented = iter.u_vecf;

        std::vector<double> foc_vec = par_img->ret_focal();
        double ang_ = CV_PI / 16.0;
        //ang_ = ang_*ang_;
        double foc;
        for(int i = 0; i < (par_img->adj).rows;i++){
            foc = foc_vec[i];
            foc = foc*.001;
            //foc = foc *foc;

            Eigen::VectorXd augmvec(6);
            augmvec <<foc,foc,foc,ang_,ang_,ang_;

            for(int j = 0 ; j < 6;j++){

                iter.u_vecf_augmented.diagonal()[6*i + j] = iter.u_vecf_augmented.diagonal()[6*i + j]*(1 + iter.lambda * augmvec[j]);

            }

        }

        if(iter.Y_vec.size() != iter.v_vec.size()){

            iter.Y_vec.assign(iter.v_vec.size(), Eigen::MatrixXd::Zero((par_img->adj).rows * 6, 2));

        }

        //std::vector<std::thread> thread_objects;
        iter.v_vec_augmented = iter.v_vec;
        double aug_lamda = 1.0 + iter.lambda*foc;

        #pragma omp parallel for schedule(dynamic)
        for(int k = 0;k < threads_vector.size();k++){

            augment_calc(k,aug_lamda,avec,iter);

        }


  /*
        for(int k = 0;k < threads_vector.size();k++){

            std::thread tobj(&adjuster::augment_calc,this,k,aug_lamda,std::ref(avec),std::ref(iter));
            thread_objects.push_back(std::move(tobj));

        }

        for (auto& t : thread_objects) {
            t.join();
        }
*/
    }


    void adjuster::error_calc(int thread,std::vector<Eigen::MatrixXd>& thread_accumulators_wy,std::vector<Eigen::VectorXd>& thread_accumulators_YEb,const std::vector<bund::A_vec> &avec){


        for(const int & p : threads_vector[thread]){

            const bund::A_vec& a = avec[p];
            thread_accumulators_wy[thread].block<6, 6>(a.idx_i * 6, a.idx_i * 6).noalias() += iter.Y_vec[p].block<6, 2>(a.idx_i * 6,0) * iter.w_vec[p].block<6, 2>(a.idx_i * 6,0).transpose();
            thread_accumulators_wy[thread].block<6, 6>(a.idx_i * 6, a.idx_j * 6).noalias() += iter.Y_vec[p].block<6, 2>(a.idx_i * 6,0) * iter.w_vec[p].block<6, 2>(a.idx_j * 6,0).transpose();
            thread_accumulators_wy[thread].block<6, 6>(a.idx_j * 6, a.idx_i * 6).noalias() += iter.Y_vec[p].block<6, 2>(a.idx_j * 6,0) * iter.w_vec[p].block<6, 2>(a.idx_i * 6,0).transpose();
            thread_accumulators_wy[thread].block<6, 6>(a.idx_j * 6, a.idx_j * 6).noalias() += iter.Y_vec[p].block<6, 2>(a.idx_j * 6,0) * iter.w_vec[p].block<6, 2>(a.idx_j * 6,0).transpose();

            thread_accumulators_YEb[thread].segment<6>(a.idx_i * 6).noalias() += iter.Y_vec[p].block<6, 2>(a.idx_i * 6,0) * iter.eB_vec[p];
            thread_accumulators_YEb[thread].segment<6>(a.idx_j * 6).noalias() += iter.Y_vec[p].block<6, 2>(a.idx_j * 6,0) * iter.eB_vec[p];

        }

    }


    void adjuster::get_error(const std::vector<bund::A_vec> &avec){

        int num_threads = thr;
        std::vector<Eigen::MatrixXd> thread_accumulators_wy(num_threads);
        std::vector<Eigen::VectorXd> thread_accumulators_YEb(num_threads);

        for (auto& mat : thread_accumulators_wy) {
            mat = Eigen::MatrixXd::Zero(iter.Y_vec[0].rows(),iter.w_vec[0].rows());
        }
        for (auto& mat : thread_accumulators_YEb) {
            mat = Eigen::VectorXd::Zero(iter.Y_vec[0].rows());
        }

        std::vector<std::thread> thread_objects;

        Eigen::MatrixXd sum_wy = Eigen::MatrixXd::Zero(iter.Y_vec[0].rows(),iter.w_vec[0].rows());
        Eigen::VectorXd sum_YEb = Eigen::VectorXd::Zero(iter.Y_vec[0].rows());

        #pragma omp parallel for schedule(dynamic)
        for(int k = 0;k < threads_vector.size();k++){

            error_calc(k,thread_accumulators_wy,thread_accumulators_YEb,avec);

        }

        /*
        for(int k = 0;k < threads_vector.size();k++){

            std::thread tobj(&adjuster::error_calc,this,k,std::ref(thread_accumulators_wy),std::ref(thread_accumulators_YEb),std::ref(avec));

            thread_objects.push_back(std::move(tobj));

        }

        for (auto& t : thread_objects) {
            t.join();
        }
*/

        for (auto& accum : thread_accumulators_wy) {
            sum_wy.noalias() += accum;
        }

        for (auto& accum : thread_accumulators_YEb) {
            sum_YEb.noalias() += accum;
        }


        Eigen::MatrixXd uvecf = iter.u_vecf_augmented;


        uvecf = uvecf - sum_wy;

        iter.delta_a = uvecf.colPivHouseholderQr().solve(iter.eA_vec - sum_YEb);

        if(iter.delta_b.size() != iter.v_vec_augmented.size()){

            iter.delta_b.resize(iter.v_vec_augmented.size());

        }

        for (int p = 0;p < iter.v_vec_augmented.size();p++){

            iter.delta_b[p] = iter.v_vec_augmented[p] * (iter.eB_vec[p] - iter.w_vec[p].transpose() * iter.delta_a) ;

        }

    }


    struct inter_par adjuster::iterate(){

        double eps = 1e-2;
        float error_start;
        float error_new;
        float lambda = iter.lambda;

        std::vector<Eigen::VectorXd> ms = par_img -> ret_measurements();
        std::vector<Eigen::VectorXd> new_error = par_er -> error(ms);

        std::vector<int> idx(ms.size());
        std::iota(idx.begin(), idx.end(), 0);
        thread_parts = idx;

        threads_vector = util::splitVector(thread_parts, thr);
        std::vector<int> sizes(threads_vector.size());

        sizes[0] = 0;
        int c = 0;
        for(int s = 1;s < sizes.size();s++){
            c = c + threads_vector[s - 1].size();
            sizes[s] = c;
        }
        thread_sizes = sizes;

        std::vector<bund::A_vec> avec = get_iter_par();

        augment(avec);

        //while(1){get_error(avec);}
        get_error(avec);

        error_start = 0;
        for (Eigen::VectorXd e : iter.e_vec){

            error_start = error_start + e.norm();

        }
        std::cout<<"\n" <<"error "<<error_start/iter.e_vec.size()<<"\n";

        int break_counter = 0;

        for (int it = 0;it<50;it++){

            std::vector<bund::A_vec> avec = get_iter_par();

            augment(avec);

            get_error(avec);

            error_start = 0;
            for (Eigen::VectorXd e : iter.e_vec){

                error_start = error_start + e.norm();

            }


            par_img -> add_delta(iter.delta_b,iter.delta_a);

            std::vector<Eigen::VectorXd> ms = par_img -> ret_measurements_saved();
            std::vector<Eigen::VectorXd> new_error = par_er -> error(ms);

            error_new = 0;
            for (Eigen::VectorXd e : new_error){

                error_new = error_new + e.norm();

            }


            error_value = error_new;
            //std::cout<<"\n" <<"test error: " << error_new<<"\n";
            if( error_start > error_new ){

                iter.lambda = iter.lambda / 10.0;
                std::cout <<"lambda: "<< iter.lambda<< " new error: " << error_new/iter.e_vec.size()<<"\n";
                error_value = error_new;
                error_start = error_new;
                par_img -> accept();
                break_counter = 0;

            }else{

                iter.lambda = iter.lambda * 10.0;
                break_counter++;
            }

            if (break_counter > 5){
                break;
            };

            lambda = iter.lambda;
            iter = inter_par();
            iter.lambda = lambda;

        }

        iter.focal = par_img -> ret_focal();
        iter.hom = par_img -> ret_hmat();
        iter.error = error_new;
        return iter;

    }


    std::vector<Eigen::MatrixXd> adjuster::ret_K(){

        return par_img -> ret_K();

    }

    std::vector<Eigen::MatrixXd> adjuster::ret_rot(){

        return par_img -> ret_rot();

    }



}


