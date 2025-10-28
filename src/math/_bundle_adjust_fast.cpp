

#include "_bundle_adjust_fast.h"

namespace bundf {

    adjuster::adjuster(const std::vector<util::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,float lmbd,const imgm::pan_img_transform &Tr,int threads){

        thr = threads;
        par_er = std::make_shared<class bund::E_func>(kp,match,*Tr.adj);
        par_img = std::make_shared<class bund::parameters>(kp,match,Tr,thr);
        iter.lambda = lmbd;
        iter.hom = par_img -> ret_hmat();

    }


    std::vector<bund::A_vec> adjuster::get_iter_par() {

        auto avec_fut = std::async(std::launch::async, [this]() {
            return par_img->ret_A_i();
        });

        auto ms_fut = std::async(std::launch::async, [this]() {
            return par_img->ret_measurements();
        });

        std::vector<bund::A_vec> avec = avec_fut.get();
        int size_tot = avec[0].size * 6;
        std::vector<Eigen::VectorXd> ms = ms_fut.get();

        auto uf_fut = std::async(std::launch::async, [this, &avec, size_tot]() {
            return ret_uf(avec, size_tot);
        });

        auto evec_fut = std::async(std::launch::async, [this, &ms]() {
            return par_er->error(ms);
        });

        iter.e_vec = evec_fut.get();

        auto eA_fut = std::async(std::launch::async, [this, &avec]() {
            return ret_Ea(avec, iter.e_vec);
        });

        iter.u_vecf = uf_fut.get();
        iter.eA_vec = eA_fut.get();

        return avec;
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

        for (auto& mat : thread_accumulators) {
            mat = Eigen::MatrixXd::Zero(total_cols, total_cols);
        }

        #pragma omp parallel for schedule(dynamic)
        for(int k = 0;k < threads_vector.size();k++){

            calc_uf(k,avec,thread_accumulators);

        }

        Eigen::MatrixXd U = Eigen::MatrixXd::Zero(total_cols, total_cols);
        for (auto const & accum : thread_accumulators) {
            U.noalias() += accum;
        }

        return U;
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
        Eigen::VectorXd Ev = Eigen::VectorXd::Zero(total_cols);

        #pragma omp parallel for schedule(dynamic)
        for(int k = 0;k < threads_vector.size();k++){

            ret_Ea_calc(k,avec,e_vec,thread_Ev);

        }

        for (auto const & vec : thread_Ev) {
            Ev.noalias() += vec;
        }

        return Ev;
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

    }


    void adjuster::get_error(const std::vector<bund::A_vec> &avec){

        iter.delta_a = iter.u_vecf_augmented.colPivHouseholderQr().solve(iter.eA_vec);

    }


    struct bundm::inter_par adjuster::iterate(){

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

            const std::vector<Eigen::VectorXd> delta_b;
            par_img -> add_delta(delta_b,iter.delta_a,false);

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
            iter = bundm::inter_par();
            iter.lambda = lambda;

        }

        iter.focal = par_img -> ret_focal();
        iter.hom = par_img -> ret_hmat();
        iter.error = error_new;
        return iter;

    }


    std::vector<Eigen::MatrixXd> adjuster::ret_K() const{

        return par_img -> ret_K();

    }

    std::vector<Eigen::MatrixXd> adjuster::ret_rot() const{

        return par_img -> ret_rot();

    }



}


