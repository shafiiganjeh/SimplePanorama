
#include "_gain_compensation.h"

namespace gain {



std::pair<int,float> get_overlapp_intensity(const cv::Mat &base, const cv::Mat &attach,const cv::Matx33f &H){

            maths::translation Tr;
            std::vector<cv::Vec2f> cor;
            cv::Matx33f T;
            cv::Mat g_base;
            cv::Mat g_attach;

            cv::cvtColor(attach, g_attach, cv::COLOR_BGR2GRAY);
            cv::cvtColor(base, g_base, cv::COLOR_BGR2GRAY);

            Tr = maths::get_translation(base, attach,H);

            T = Tr.T * H;

            cv::Mat panorama;
            cv::warpPerspective(g_attach, panorama, T, cv::Size(Tr.xend - Tr.xstart + 1, Tr.yend - Tr.ystart + 1), cv::INTER_LINEAR);
            cv::Mat roi(panorama, cv::Rect(-Tr.xstart,-Tr.ystart,base.cols, base.rows));
            cv::Mat mask_a;
            cv::Mat mask_b;

            cv::threshold( roi, mask_a, 1, 1, 0 );
            cv::threshold( g_base, mask_b, 1, 1, 0 );

            cv::bitwise_and(mask_a,mask_b,mask_a);

            float area = cv::countNonZero(mask_a);

            cv::Mat overlapp = roi.mul(mask_a);

            int intensity_sum = cv::sum(overlapp)[0];

            std::pair<int,float> ret = std::pair<int,float>(area,(float)intensity_sum/area);

            return ret;
    }


    float A(const float& S_g_exp,const float& S_N_exp,const int& i ,const std::vector<std::vector<int>>& N_ij,const std::vector<std::vector<float>>& I_ij,const cv::Mat& adj){

            float ret = 0;

            for (int j = 0; j < adj.cols;j++){

                if ((i == j) or (.5 > adj.at<double>(i,j))){

                    float ret = ret + 0;

                }else{

                    ret = ret + (float)N_ij[i][j]*( ( 2*I_ij[i][j]*I_ij[j][i] )/S_N_exp - 1/S_g_exp );

                }

            }
            return ret;
    }


    float B(const float& S_N_exp,const int& i ,const int& j,const std::vector<std::vector<int>>& N_ij,const std::vector<std::vector<float>>& I_ij,const cv::Mat& adj){

            float ret;

            if ((i == j) or (.5 > adj.at<double>(i,j))){

                ret = 0;

            }else{

                ret = -(float)N_ij[i][j]*( ( I_ij[i][j]*I_ij[j][i] + I_ij[j][i]*I_ij[j][i] ) / S_N_exp );

            }

            return ret;
    }


    float gain_vec(const int& i,const float& S_g_exp,const std::vector<std::vector<int>>& N_ij,const cv::Mat& adj){

        float ret = 0;
        float c = 0;

        for (int j = 0; j < adj.rows; j++){

            if ((i == j) or (.5 > adj.at<double>(i,j))){

                ret = ret + 0;

            }else{
                c = c+ 1;
                ret = ret - (float)N_ij[i][j]/S_g_exp;

            }

        }
        std::cout<<"\n"<<"index: " << i<<" count: "<< c<<"\n";
        return ret;

    }

    std::pair<Eigen::VectorXf,Eigen::MatrixXf> set_up_equations(std::vector<cv::Mat> &imags,const cv::Mat& adj,std::vector<std::vector< cv::Matx33f >>& Hom_mat){

            float S_N_exp = 100;
            float S_g_exp = .01;
            int size = imags.size();

            std::vector<std::vector<int>> N_ij(size, std::vector<int>(size, 0));
            std::vector<std::vector<float>> I_ij(size, std::vector<float>(size, 0));

            for (int i = 0;i < size;i++){
                for (int j = 0;j < size;j++){

                    if ((i == j) or (.5 > adj.at<double>(i,j))){

                        N_ij[i][j] = 0;
                        I_ij[i][j] = 0;

                    }else{

                        std::pair<int,float> N_I = get_overlapp_intensity(imags[i],imags[j],Hom_mat[i][j]);
                        N_ij[i][j] = N_I.first;
                        I_ij[i][j] = N_I.second;

                    }

                }
            }

            Eigen::VectorXf gain(size);
            Eigen::MatrixXf error_matrix(size,size);

            for (int i = 0;i < size;i++){

                gain(i) = gain_vec(i,S_g_exp,N_ij,adj);

                for (int j = 0;j < size;j++){

                    if (i == j){

                        error_matrix(i,j) = A(S_g_exp,S_N_exp,i ,N_ij,I_ij,adj);

                    }else{

                        error_matrix(i,j) = B(S_N_exp,i ,j,N_ij, I_ij,adj);

                    }
                }
            }

            return std::pair<Eigen::VectorXf,Eigen::MatrixXf>(gain,error_matrix);
    }

    std::vector<float> gain_compensation(std::vector<cv::Mat> &imags,const cv::Mat& adj,std::vector<std::vector< cv::Matx33f >>& Hom_mat){

        std::pair<Eigen::VectorXf,Eigen::MatrixXf> G;
        G = gain::set_up_equations(imags,adj,Hom_mat);

        Eigen::VectorXf x = G.second.colPivHouseholderQr().solve(G.first);

        std::cout <<"g vector: "<< x<<"-";
        //if (x.maxCoeff() > 1) x = x / x.maxCoeff();
        std::cout<<"\n"<<"vector: " << x<<"\n";
        std::vector<float> ret(x.data(), x.data() + x.size());
        return ret;

    }

}
