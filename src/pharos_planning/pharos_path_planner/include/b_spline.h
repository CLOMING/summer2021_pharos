#ifndef _B_SPLINE_H_
#define _B_SPLINE_H_

void B_spline(const nav_msgs::PathConstPtr& input_path, nav_msgs::PathPtr& splined_path){


        int n = input_path->poses.size(); //number of control points
        uint32_t K = B_Spline_Order; //order
        if(n<B_Spline_Order){
            K = n;
        }
        //printf("n: %d, K: %d\n",n,K);

        std::vector<double> T; //knots
        int m = n+K; //number of knots
        T.resize(m);
        std::vector<std::vector<double> > B(m,std::vector<double>(K,0.0)); //Basis function
        if(n<K){
            ROS_ERROR("Number of Control Points(input path) have to be bigger than order k(k = %d)",K);
            return;
        }

        if(K<2){
            ROS_ERROR("Order of B-spline, 'k' have to be bigger than 1!");
            return;
        }

        for(int i=0;i<K;i++){
            T[i] = 0.0;
        }
        for(int i=K;i<m-K+1;i++){
            T[i] = T[i-1] + 1.0;
        }
        for(int i=m-K+1;i<m;i++){
            T[i] = T[i-1];
        }

//		printf("T: ");
//		for(int i=0;i<T.size();i++){
//			printf("%.1f ",T[i]);
//		}
//		printf("\n");

        double dt = B_Spline_Dt;
        double t = T[0];

        while(t <= (T[m-1]+0.0001)){
            for(int j=0;j<K;j++) {
                for (int i=0; i < n+K-1; i++) {
                    if(j==0) {
                        if ((fabs(t-T[i])<0.0001) ||(t > T[i])){
                            if(t < (T[i + 1]-0.0001)){
                                B[i][j] = 1.0;
                            }
                            else B[i][j] = 0.0;
                        }
                        else B[i][j] = 0.0;
                        //printf("t: %.2f, T[%d]: %.2f,B[%d][%d] = %.3f\n",t,i,T[i],i,j,B[i][j]);
                    }
                    else{
                        double r1 = 0.0;
                        double r2 = 0.0;
                        //if(fabs(T[i+j-1]-T[i])<0.0001){
                        if(fabs(T[i+j]-T[i])<0.0001){
                            r1 = 0.0;
                        }
                        else{
                            //r1 = (t-T[i])/(T[i+j-1]-T[i]);
                            r1 = (t-T[i])/(T[i+j]-T[i]);
                        }
                        //if(fabs(T[i+j]-T[i+1])<0.0001){
                        if(fabs(T[i+j+1]-T[i+1])<0.0001){
                            r2 = 0.0;
                        }
                        else{
                            //r2 = (T[i+j]-t)/(T[i+j]-T[i+1]);
                            r2 = (T[i+j+1]-t)/(T[i+j+1]-T[i+1]);
                        }
                        B[i][j] = r1*B[i][j-1] + r2*B[i+1][j-1];
                        if(B[i][j] == 0){
                            //printf("B[%d][%d]: %.1f, r1:%.2f ,r2:%.2f ,B[%d][%d]: %.2f,B[%d][%d]: %.2f\n",i,j,B[i][j],r1,r2,i,j-1,B[i][j-1],i+1,j-1,B[i+1][j-1]);
                        }
                        //printf("B[%d][%d] = %.3f\n",i,j,B[i][j]);
                    }
                }
                n-=1;
            }
            n = input_path->poses.size();

            if(fabs(t-T[0])<0.0001){
                B[0][K-1] = 1.0;
            }
            if(fabs(t - T[m-1])<0.0001){
                //printf("t: %.3f, T[m-1]: %.3f\n",t,T[m-1]);
                B[n-1][K-1] = 1.0;
            }

            geometry_msgs::PoseStamped s;
            for(int k=0;k<m-K;k++){
                s.pose.position.x += input_path->poses.at(k).pose.position.x * B[k][K-1];
                s.pose.position.y += input_path->poses.at(k).pose.position.y * B[k][K-1];
                //s.pose.position.z += input_path->poses.at(k).pose.position.z * B[k][K-1];
                //printf("B[%d][%d] (%.1f) = %.3f\n",k,K-1,t,B[k][K-1]);
            }
            splined_path->poses.push_back(s);
            t+=dt;
        }

        for(unsigned int i=0; i<splined_path->poses.size()-1;i++){
            double x1,x2,y1,y2;
            x1 = splined_path->poses.at(i).pose.position.x;
            y1 = splined_path->poses.at(i).pose.position.y;
            x2 = splined_path->poses.at(i+1).pose.position.x;
            y2 = splined_path->poses.at(i+1).pose.position.y;

            double heading = atan2(y2-y1,x2-x1);

            geometry_msgs::Quaternion quat_heading;
            tf::Quaternion q;
            q.setRPY(0,0,heading);
            tf::quaternionTFToMsg(q,quat_heading);
            splined_path->poses.at(i).pose.orientation = quat_heading;
        }
        //(*splined_path->poses.begin()).pose.orientation = splined_path->poses.at(1).pose.orientation;
        (*splined_path->poses.end()).pose.orientation = splined_path->poses.at(splined_path->poses.size()-2).pose.orientation;

        for(unsigned int i=0; i<splined_path->poses.size();i++){
            tf::Pose pose;
            tf::poseMsgToTF(splined_path->poses.at(i).pose, pose);
            double Roll,Pitch, Yaw;
            pose.getBasis().getRPY(Roll, Pitch, Yaw);
            double vehicle_heading = Yaw;
            //printf("splined_path_heading [%d]: %.3f\n",i,vehicle_heading);
        }
        //printf("Splined path size: %d\n",splined_path->poses.size());
    };

#endif

