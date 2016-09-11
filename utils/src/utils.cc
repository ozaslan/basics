#include "utils.hh"

using namespace arma;

namespace utils{

	void get_ellipse_parameters(vector<double> &fit, Ellipse &ell)
	{
		double a, b, c, d, e, f;
		//###a = ellipse(1);	b = ellipse(2);	c = ellipse(3);	
		//###d = ellipse(4);	e = ellipse(5);	f = ellipse(6);
		a = fit[0]; b = fit[1]; c = fit[2];
		d = fit[3]; e = fit[4]; f = fit[5];

		//return;
		//###A = [a b/2; b/2 c];
		static mat A(2,2);
		A(0,0) = a; A(0,1) = b/2;
		A(1,0) = b/2; A(1,1) = c;
		//###B = [d; e];
		static mat B(2,1);
		B(0,0) = d; B(1,0) = e;
		//###K = -A^-1*B/2;
		vec K = -inv(A) * B / 2;
		//###M = A / (B'*A^-1*B/4-f);
		mat temp = B.t() * inv(A) * B;
		mat M = A / (temp(0,0) / 4 - f);
		//###[vs es] = eig(M);
		cx_vec cx_eigval;
		mat r_eigvec, l_eigvec, eigval;
		eig_gen(cx_eigval, l_eigvec, r_eigvec, M);
		eigval = real(cx_eigval);
		/*
		   {
		   M.print("Before M = ");
		   r_eigvec.print("Before eig = ");
		   }
		 */
		if(eigval(0) > eigval(1))
		{
			l_eigvec = r_eigvec;
			r_eigvec(0,0) = l_eigvec(0,1);
			r_eigvec(1,0) = l_eigvec(1,1);
			r_eigvec(0,1) = l_eigvec(0,0);
			r_eigvec(1,1) = l_eigvec(1,0);
			double temp = eigval(1);
			eigval(1) = eigval(0);
			eigval(0) = temp;
		}
		/*
		   {
		   M.print("M = ");
		   r_eigvec.print("eig = ");
		   }
		 */
		//return;
		//###theta = (atan2(vs(1,2), vs(1,1)));
		double theta = atan2(r_eigvec(1,0), r_eigvec(0,0));

		//if theta <= pi/2 && theta >= -pi/2
		//elseif theta > pi/2
		//	theta = theta - pi;
		//elseif theta < -pi/2
		//	theta = pi + theta;
		//end
		//ROS_INFO("THETA BEFORE : %lf", theta / PI * 180);
		if(theta < PI/2 && theta >= -PI/2) {}
		else if(theta > PI/2)
		{
			theta = -(PI - theta);
		}
		else if(theta < -PI/2)
		{
			theta = PI + theta;
		}
		//ROS_INFO("THETA AFTER : %lf", theta / PI * 180);

		ell.center = K;             //% center of the ellipse
		ell.len_major = 1/sqrt(eigval(0,0)); //% length of major axis
		ell.len_minor = 1/sqrt(eigval(1,0)); //% length of minor axis
		ell.major_axis(0) = r_eigvec(0,0);
		ell.major_axis(1) = r_eigvec(1,0);          //% unit vector along major axis
		if (ell.major_axis(0) < 0)
			ell.major_axis = -ell.major_axis;
		ell.minor_axis(0) = r_eigvec(0,1);//% unit vector along major axis
		ell.minor_axis(1) = r_eigvec(1,1);
		if (ell.minor_axis(0) < 0)
			ell.minor_axis = -ell.minor_axis;
		ell.theta = theta;          //% angle between major axis and positive x-axis
		//ROS_INFO("BEFORE ell.theta = %lf\n", ell.theta / PI * 180);
		//ell.theta = (PI/2 - fabs(ell.theta)) * (ell.theta < 0 ? -1 : 1);
		//ROS_INFO("AFTER ell.theta = %lf\n", ell.theta / PI * 180);

		//printf("center = [%f,%f]\n", ell.center(0), ell.center(1));
		//printf("lenght of axes (M,m) = [%f,%f]\n", ell.len_major, ell.len_minor);
		//printf("major axis = [%f,%f]\n", ell.major_axis(0), ell.major_axis(1));
		//printf("minor axis = [%f,%f]\n", ell.minor_axis(0), ell.minor_axis(1));
		//printf("theta = [%f]\n", ell.theta / PI * 180);

	}

	void fit_ellipse(vector<double> &xs, vector<double> &ys, vector<double> &fit)
	{
		double mean_x = 0 , mean_y = 0;
		// centroid = mean(XY);   % the centroid of the data set
		for(int i = 0 ; i < (int)xs.size() ; i++)
		{
			mean_x += xs[i];
			mean_y += ys[i];
		}
		mean_x /= xs.size();
		mean_y /= ys.size();

		// D1 = [(XY(:,1)-centroid(1)).^2, (XY(:,1)-centroid(1)).*(XY(:,2)-centroid(2)), (XY(:,2)-centroid(2)).^2];
		// D2 = [XY(:,1)-centroid(1), XY(:,2)-centroid(2), ones(size(XY,1),1)];
		//ros::Time start, end;

		//start = ros::Time::now();

		static mat D1, D2;
		if(D1.n_rows != xs.size())
			D1.resize(xs.size(), 3);
		if(D2.n_rows != xs.size())
			D2.resize(xs.size(), 3);
		//mat D1(xs.size(), 3); // N-3
		//mat D2(xs.size(), 3); // N-3
		for(int i = 0 ; i < (int)xs.size() ; i++)
		{
			double dx = (xs[i]-mean_x);
			double dy = (ys[i]-mean_y);
			D1(i,0) = dx * dx;
			D1(i,1) = dx * dy;
			D1(i,2) = dy * dy;

			D2(i,0) = dx;
			D2(i,1) = dy;
			D2(i,2) = 1;
		}

		//end = ros::Time::now();
		//printf("      D1, D2 preperation : %lf\n", (end - start).toSec());	
		//start = ros::Time::now();

		//D1.print("D1 = ");
		//D2.print("D2 = ");
		//S1 = D1'*D1;
		//S2 = D1'*D2;
		//S3 = D2'*D2;
		//T = -inv(S3)*S2';
		//M = S1 + S2*T;
		mat S1 = D1.t() * D1; // 3-3
		mat S2 = D1.t() * D2; // 3-3
		mat S3 = D2.t() * D2; // 3-3
		//S1.print("S1 = ");
		//S2.print("S2 = ");
		//S3.print("S3 = ");
		mat T = -inv(S3) * S2.t(); // 3-3
		mat M = S1 + S2 * T; // 3-3
		//T.print("T = ");
		//M.print("M = ");
		/*
		   {
		   printf("myS1 = [%.5f, %.5f, %.5f; \n", S1(0,0), S1(0,1), S1(0,2));
		   printf("      %.5f, %.5f, %.5f; \n", S1(1,0), S1(1,1), S1(1,2));
		   printf("      %.5f, %.5f, %.5f];\n", S1(2,0), S1(2,1), S1(2,2));
		   printf("myS2 = [%.5f, %.5f, %.5f; \n", S2(0,0), S2(0,1), S2(0,2));
		   printf("      %.5f, %.5f, %.5f; \n", S2(1,0), S2(1,1), S2(1,2));
		   printf("      %.5f, %.5f, %.5f];\n", S2(2,0), S2(2,1), S2(2,2));
		   printf("myS3 = [%.5f, %.5f, %.5f; \n", S3(0,0), S3(0,1), S3(0,2));
		   printf("      %.5f, %.5f, %.5f; \n", S3(1,0), S3(1,1), S3(1,2));
		   printf("      %.5f, %.5f, %.5f];\n", S3(2,0), S3(2,1), S3(2,2));
		   printf("myT  = [%.5f, %.5f, %.5f; \n",  T(0,0),  T(0,1),  T(0,2));
		   printf("      %.5f, %.5f, %.5f; \n",  T(1,0),  T(1,1),  T(1,2));
		   printf("      %.5f, %.5f, %.5f];\n",  T(2,0),  T(2,1),  T(2,2));
		   printf("myM  = [%.5f, %.5f, %.5f; \n",  M(0,0),  M(0,1),  M(0,2));
		   printf("      %.5f, %.5f, %.5f; \n",  M(1,0),  M(1,1),  M(1,2));
		   printf("      %.5f, %.5f, %.5f];\n" ,  M(2,0),  M(2,1),  M(2,2));
		   }
		 */
		//M = [M(3,:)./2; -M(2,:); M(1,:)./2];
		for(int i = 0 ; i < 3 ; i++)
		{
			double c1, c2, c3;
			c1 = M(0, i);
			c2 = M(1, i);
			c3 = M(2, i);
			M(0, i) = c3/2;
			M(1, i) = -c2;
			M(2, i) = c1/2;
		}

		//end = ros::Time::now();
		//printf("      S1, S2, S3 calculation : %lf\n", (end - start).toSec());	

		//start = ros::Time::now();
		/*
		   {
		   printf("myM  = [%.5f, %.5f, %.5f; \n",  M(0,0),  M(0,1),  M(0,2));
		   printf("      %.5f, %.5f, %.5f; \n",  M(1,0),  M(1,1),  M(1,2));
		   printf("      %.5f, %.5f, %.5f];\n" , M(2,0),  M(2,1),  M(2,2));
		   }
		 */
		cx_vec eigval;
		//cx_mat r_eigvec, l_eigvec;
		mat r_eigvec, l_eigvec;
		eig_gen(eigval, l_eigvec, r_eigvec, M);
		//eig_gen(eigval, l_eigvec, r_eigvec, M);
		mat& eigvec = r_eigvec;
		/*
		   {
		   printf("myeigvec = [%.5f, %.5f, %.5f; \n", eigvec(0,0), eigvec(0,1), eigvec(0,2));
		   printf("          %.5f, %.5f, %.5f; \n", eigvec(1,0), eigvec(1,1), eigvec(1,2));
		   printf("          %.5f, %.5f, %.5f];\n"   , eigvec(2,0), eigvec(2,1), eigvec(2,2));
		   }
		 */
		// cond = 4*evec(1,:).*evec(3,:)-evec(2,:).^2;
		vec cond(3);
		cond(0) = 4 * (eigvec(0,0) * eigvec(2,0)) - (eigvec(1,0) * eigvec(1,0));
		cond(1) = 4 * (eigvec(0,1) * eigvec(2,1)) - (eigvec(1,1) * eigvec(1,1));
		cond(2) = 4 * (eigvec(0,2) * eigvec(2,2)) - (eigvec(1,2) * eigvec(1,2));
		/*
		   {
		   printf("mycond = [%.5f, %.5f, %.5f];\n"   , cond(0), cond(1), cond(2));
		   }
		 */
		// A1 = evec(:,find(cond>0));
		mat A(6,1), A1; // 6-1, 3-1
		if(cond(0) > 0)
			A1 = eigvec.col(0);
		else if(cond(1) > 0)
			A1 = eigvec.col(1);
		else
			A1 = eigvec.col(2);
		//ROS_INFO("A1 Size : [%d,%d]\n", A1.n_rows, A1.n_cols);
		// A = [A1; T*A1];
		A(0) = A1(0);
		A(1) = A1(1);
		A(2) = A1(2);
		vec temp = T * A1;
		A(3) = temp(0);
		A(4) = temp(1);
		A(5) = temp(2);

		//end = ros::Time::now();
		//printf("      eigen value decomp : %lf\n", (end - start).toSec());	

		/*
		   {
		   printf("myA = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f];\n", A(0), A(1), A(2), A(3), A(4), A(5));
		   }
		 */
		//A4 = A(4)-2*A(1)*centroid(1)-A(2)*centroid(2);
		//A5 = A(5)-2*A(3)*centroid(2)-A(2)*centroid(1);
		//A6 = A(6) + 
		//	   A(1)*centroid(1)^2 + 
		//     A(3)*centroid(2)^2 +
		//	   A(2)*centroid(1)*centroid(2) - 
		//     A(4)*centroid(1) - 
		//     A(5)*centroid(2);
		double A4 = A(3) - 2 * A(0) * mean_x - A(1) * mean_y;
		double A5 = A(4) - 2 * A(2) * mean_y - A(1) * mean_x;
		double A6 = A(5) + 	A(0) * mean_x * mean_x + 
			A(2) * mean_y * mean_y +
			A(1) * mean_x * mean_y - 
			A(3) * mean_x -
			A(4) * mean_y;
		// A(4) = A4;  A(5) = A5;  A(6) = A6;
		A(3) = A4;  A(4) = A5;  A(5) = A6;
		/*
		   {
		   printf("myA = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f];\n", A(0), A(1), A(2), A(3), A(4), A(5));
		   }
		 */
		A = A/norm(A, 2);
		/*
		   {
		   printf("myA = [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f];\n", A(0), A(1), A(2), A(3), A(4), A(5));
		   }
		 */
		if(fit.size() != 6)
			fit.resize(6);
		for(int i = 0 ; i < 6 ; i++)
			fit[i] =A(i);
		/*
### centroid = mean(XY);   % the centroid of the data set

###D1 = [(XY(:,1)-centroid(1)).^2, (XY(:,1)-centroid(1)).*(XY(:,2)-centroid(2)),...
(XY(:,2)-centroid(2)).^2];
###D2 = [XY(:,1)-centroid(1), XY(:,2)-centroid(2), ones(size(XY,1),1)];
###S1 = D1'*D1;
###S2 = D1'*D2;
###S3 = D2'*D2;
###T = -inv(S3)*S2';
###M = S1 + S2*T;
###M = [M(3,:)./2; -M(2,:); M(1,:)./2];
###[evec,eval] = eig(M);
###cond = 4*evec(1,:).*evec(3,:)-evec(2,:).^2;
###A1 = evec(:,find(cond>0));
###A = [A1; T*A1];
###A4 = A(4)-2*A(1)*centroid(1)-A(2)*centroid(2);
###A5 = A(5)-2*A(3)*centroid(2)-A(2)*centroid(1);
###A6 = A(6)+A(1)*centroid(1)^2+A(3)*centroid(2)^2+...
###	 A(2)*centroid(1)*centroid(2)-A(4)*centroid(1)-A(5)*centroid(2);
### A(4) = A4;  A(5) = A5;  A(6) = A6;
### A = A/norm(A);
		 */
	}

	bool fit_ellipse_ransac(vector<double> &xs, vector<double> &ys, vector<double> &fit, int num_trials)
	{
		//printf("Got to fit_ellipse_ransac\n");
		//int num_trials = 200;
		int num_model_points = 6;
		double best_err = 999999999;

		static vector<bool>  selected_points(xs.size());
		static vector<bool>  valid_points(xs.size());
		vector<int>   valid_points_index_map;
		valid_points_index_map.reserve(xs.size());
		static vector<double> model_xs(num_model_points); 
		static vector<double> model_ys(num_model_points);
		static vector<double> best_model_xs(num_model_points); 
		static vector<double> best_model_ys(num_model_points);

		if(selected_points.size() != xs.size())
			selected_points.resize(xs.size());
		if(valid_points.size() != xs.size())
			valid_points.resize(xs.size());

		//ros::Time start, end;

		//start = ros::Time::now();

		for(int i = 0 ; i < (int)valid_points.size() ; i++)
		{
			if( xs[i] == +std::numeric_limits<double>::infinity() || 
					ys[i] == +std::numeric_limits<double>::infinity() ||
					xs[i] == -std::numeric_limits<double>::infinity() || 
					ys[i] == -std::numeric_limits<double>::infinity() ||
					xs[i] !=  xs[i] ||
					ys[i] !=  ys[i])
			{
				valid_points[i] = false;
			}
			else
				valid_points[i] = true;

			//printf("xs[%d] = %lf, ys[%d] = %lf\n", i, xs[i], i, ys[i]);
			//printf("valid_points[%d] = %s\n", i, valid_points[i] ? "TRUE" : "FALSE");		
		}

		for(int i = 0 ; i < (int)valid_points.size() ; i++)
		{
			if(valid_points[i] == false)
				continue;			
			valid_points_index_map.push_back(i);
		}

		//end = ros::Time::now();
		//printf("      valid_points[] preperation : %lf\n", (end - start).toSec());	

		//printf("We have %d valid points\n", valid_points_index_map.size());

		if(fit.size() != 6)
			fit.resize(6);

		for(int trial = 0 ; trial < num_trials ; trial++)
		{
			//start = ros::Time::now();
			for(int i = 0 ; i < (int)selected_points.size() ; i++)
				selected_points[i] = false;
			//printf("11\n");
			// Select random N model points to make a first guess for model
			for (int i = 0 ; i < num_model_points ; i++)
			{
				//printf("12\n");
				int idx = (rand()/(double)RAND_MAX) * (valid_points_index_map.size() - 1);
				idx = valid_points_index_map[idx];
				//int idx = (rand()/(double)RAND_MAX) * (xs.size() - 1) / num_model_points;
				//idx += (double)i / num_model_points * (xs.size() - 1);
				//idx = idx > xs.size() - 1 ? xs.size() - 1 : idx;
				//printf("idx = %d\n", idx);
				while(selected_points[idx] == true || valid_points[idx] == false)
				{
          //cout << "selected_points[" << idx << "] = " << selected_points[idx] << endl;
          //cout << "valid_points[" << idx << "] = " <<  valid_points[idx] << endl;
          //cout << "xs.size() = " << xs.size() << endl;
          idx++;
					idx = (idx) % xs.size();
				}
				selected_points[idx] = true;
				//printf("14.1\n");
				model_xs[i] = xs[idx];
				//printf("14.2\n");
				model_ys[i] = ys[idx];
				//printf("15\n");
				//printf("idx = %d, valid_points[%d] = %s\n", idx, idx, valid_points[idx] ? "TRUE" : "FALSE");
				//printf("Model_xs[%d] = %f\n", i, model_xs[i]);
				//printf("Model_ys[%d] = %f\n", i, model_ys[i]);
			}
			//printf("18\n");
			// Fit an ellipse with model data
			/*for(int i = 0 ; i < model_xs.size() ; i++)
			  {
			  printf("model_xs[%d] = %lf\n", i, model_xs[i]);
			  }*/
			fit_ellipse(model_xs, model_ys, fit);
			//printf("19\n");
			// Calculate the total error

			//end = ros::Time::now();
			//printf("      fit_ellipse(model_xs, model_ys, fit); : %lf\n", (end - start).toSec());	
			//start = ros::Time::now();

			double total_err = 0;
			for(int i = 0 ; i < (int)xs.size() ; i++)
			{
				if( valid_points[i] == false)
					continue;
				double err =	
					fit[0] * xs[i] * xs[i] +
					fit[1] * xs[i] * ys[i] +
					fit[2] * ys[i] * ys[i] +
					fit[3] * xs[i] +
					fit[4] * ys[i] +
					fit[5];
				total_err += fabs(err);
			}
			//printf("20 - total_err = %lf\n", total_err);
			if(total_err < best_err)
			{
				best_err = total_err;
				best_model_xs = model_xs;
				best_model_ys = model_ys;
			}		

			//end = ros::Time::now();
			//printf("      total_err calculation : %lf\n", (end - start).toSec());	
			//printf("21\n");
		}
		//printf("22\n");
		/*for(int i = 0 ; i < best_model_xs.size() ; i++)
		  {
		  printf("best_model_xs[%d] = %lf\n", i, best_model_xs[i]);
		  }*/
		fit_ellipse(best_model_xs, best_model_ys, fit);
		//xs = best_model_xs;
		//ys = best_model_ys;
		//printf("23\n");
		//printf("Done with fit_ellipse_ransac\n");

		return true;
	}



}

