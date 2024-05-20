// #include <iostream>
// #include <vector>
// #include "../../../Eigen/Dense"
// using namespace Eigen; 


// template<typename Derived>
// void qp_lagrange(Eigen::MatrixBase<Derived>& H, Eigen::MatrixBase<Derived>& c, Eigen::MatrixBase<Derived>& A, Eigen::MatrixBase<Derived>& b,
// 	Eigen::MatrixBase<Derived>& x, Eigen::MatrixBase<Derived>& lambda, const int& dim, const int& m) {
// 	Eigen::MatrixXd G(dim, dim);
// 	Eigen::MatrixXd B(m, dim);
// 	Eigen::MatrixXd C(m, m);
 
// 	G = H.inverse() - H.inverse() * A.transpose() * (A * H.inverse() * A.transpose()).inverse() * A * H.inverse();
// 	B = (A * H.inverse() * A.transpose()).inverse() * A * H.inverse();
// 	C = -1 * (A * H.inverse() * A.transpose()).inverse();
 
// 	x = B.transpose() * b - G * c;
// 	lambda = B * c - C * b;
// }
 
// template<typename Derived>
// void qp_activeset(Eigen::MatrixBase<Derived>& H, Eigen::MatrixBase<Derived>& c, Eigen::MatrixBase<Derived>& Ae, Eigen::MatrixBase<Derived>& be,
// 	Eigen::MatrixBase<Derived>& Ai, Eigen::MatrixBase<Derived>& bi, const int& ne, const int& ni, Eigen::MatrixBase<Derived>& x) {
// 	double epsilon = 1e-9;
// 	Eigen::MatrixXd Sk = Eigen::MatrixXd::Zero(ni, 1);
// 	std::vector<double> asIndex;	//记录第几个不等式约束被选为有效集
 
// 	Eigen::MatrixXd bi_left(Ai.rows(), x.cols());
// 	bi_left = Ai * x;
// 	Eigen::MatrixXd Aee(1, 1);
 
// 	#pragma region set initial active set
// 	for (int i = 0; i < ni; i++) {
// 		double bb = bi(i, 0);
// 		if (bi_left(i, 0) <= bi(i, 0) + epsilon) {
// 			Sk(i, 0) = 1;
// 		}
// 	}
// 	#pragma endregion
 
// 	#pragma region main loop
// 	int iterTimes = 0;
// 	while (iterTimes < 150) {
// 		int activeSetSize = 0;
// 		for (int i = 0; i < Sk.rows(); i++) {
// 			if (Sk(i) == 1) {
// 				activeSetSize += 1;
// 			}
// 		}
// 		if (ne != 0) {
// 			Aee.resize(ne + activeSetSize, Ae.cols());
// 			Aee.block(0, 0, Ae.rows(), Ae.cols()) = Ae;
// 			int aee_index = 0;
// 			for (int i = 0; i < ni; i++) {
// 				if (Sk(i, 0) == 1) {
// 					Aee.block(ne + aee_index, 0, 1, Ae.cols()) = Ai.row(i);
// 					asIndex.push_back(i);
// 					aee_index += 1;
// 				}
// 			}
// 		}
// 		else {
// 			Aee.resize(activeSetSize, Ai.cols());
// 			int aee_index = 0;
// 			for (int i = 0; i < ni; i++) {
// 				if (Sk(i, 0) == 1) {
// 					Aee.block(aee_index, 0, 1, Ai.cols()) = Ai.row(i);
// 					asIndex.push_back(i);
// 					aee_index += 1;
// 				}
// 			}
// 		}
 
// 		// 解子问题
// 		Eigen::MatrixXd d = Eigen::MatrixXd::Zero(x.rows(), 1);
// 		Eigen::MatrixXd lambda = Eigen::MatrixXd::Zero(Aee.rows(), 1);
// 		Eigen::MatrixXd gk;
// 		gk = H * x + c;
// 		Eigen::MatrixXd bee = Eigen::MatrixXd::Zero(Aee.rows(), 1);
// 		qp_lagrange(H, gk, Aee, bee, d, lambda, d.rows(), Aee.rows());
 
// 		double d_norm = 0;
// 		for (int i = 0; i < d.rows(); i++) {
// 			d_norm += d(i) * d(i);
// 		}
// 		// d的模是否=0 是 停算
// 		if (d_norm < 1e-6) {
// 			// 终止判断
// 			double minLambda = 9999;
// 			int minLambdaIndex = 0;
// 			for (int i = 0; i < lambda.rows(); i++) {
// 				if (lambda(i, 0) < minLambda) {
// 					minLambda = lambda(i, 0);
// 					minLambdaIndex = i;
// 				}
// 			}
// 			if (minLambda > 0) {
// 				break;
// 			}
// 			// 从有效集中剔除约束
// 			else {
// 				int removed_cons = asIndex[minLambdaIndex - ne];
// 				Sk(removed_cons, 0) = 0;
// 			}
// 		}
// 		// 否 求步长
// 		else {
// 			Eigen::MatrixXd ad(Ai.rows(), 1);
// 			ad = Ai * d;
// 			Eigen::MatrixXd ax(Ai.rows(), 1);
// 			ax = Ai * x;
// 			std::vector<double> alphaList;
// 			int min_alpha_index = 0;
// 			double min_alpha = 9999;
// 			for (int i = 0; i < Sk.rows(); i++) {
// 				if (Sk(i) == 0 && ad(i, 0) < 0) {
// 					double alpha_ = (bi(i, 0) - ax(i, 0)) / ad(i, 0);
// 					if (alpha_ < min_alpha) {
// 						min_alpha = alpha_;
// 						min_alpha_index = i;
// 					}
// 					alphaList.push_back(alpha_);
// 				}
// 			}
 
// 			if (min_alpha > 1) {
// 				x = x + d;
// 			}
// 			else {
// 				x = x + min_alpha * d;
// 				Sk(min_alpha_index) = 1;	// 将最小的alpha对应的约束加入有效集
// 			}
// 		}
		
// 		iterTimes += 1;
// 	}
// 	#pragma endregion
// }
 
// #pragma region function
// template<typename Derived>
// double f(Eigen::MatrixBase<Derived>& x) {
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	double f = 6.0 * x1 / x2 + x2 / (x1 * x1);
// 	return f;
// }
// template<typename Derived>
// Eigen::MatrixXd h(Eigen::MatrixBase<Derived>& x) {
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	double h_ = x1 * x2 - 2;
// 	Eigen::MatrixXd h(1, 1);
// 	h << h_;
// 	return h;
// }
// template<typename Derived>
// Eigen::MatrixXd g(Eigen::MatrixBase<Derived>& x) {
// 	// 不等式要大于等于
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	Eigen::MatrixXd g(1, 1);
// 	double g_ = -1 + x1 + x2;
// 	g << g_;
// 	return g;
// }
// template<typename Derived>
// Eigen::MatrixXd gradf(Eigen::MatrixBase<Derived>& x) {
// 	// 设定f梯度
// 	Eigen::MatrixXd gf(2, 1);
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	gf << 6.0 / x2 - 2 * x2 / pow(x1, 3),
// 		-6.0 * x1 / (x2 * x2) + 1.0 / (x1 * x1);
 
// 	return gf;
// }
// template<typename Derived>
// Eigen::MatrixXd grad2f(Eigen::MatrixBase<Derived>& x) {
// 	// 设定f梯度的梯度
// 	Eigen::MatrixXd g2f(2, 2);
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	g2f << 6.0 * x2 /pow(x1, 4), -6.0 / (x2 * x2) - 2.0 / pow(x1, 3),
// 		-6.0 / (x2 * x2) - 2.0 / pow(x1, 3), 12.0 * x1 / pow(x2, 3);
 
// 	return g2f;
// }
// template<typename Derived>
// Eigen::MatrixXd gradh(Eigen::MatrixBase<Derived>& x) {
// 	// 设定h梯度
// 	Eigen::MatrixXd gh(2, 1);
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	gh << x2,
// 		x1;
// 	return gh;
// }
// template<typename Derived>
// Eigen::MatrixXd grad2h(Eigen::MatrixBase<Derived>& x) {
// 	// 设定h梯度的梯度
// 	Eigen::MatrixXd g2h(2, 2);
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	g2h << 0, 1,
// 		1, 0;
// 	return g2h;
// }
// template<typename Derived>
// Eigen::MatrixXd gradg(Eigen::MatrixBase<Derived>& x) {
// 	// 设定g梯度
// 	Eigen::MatrixXd gg(2, 1);
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	gg << 1,
// 		1;
// 	return gg;
// }
// template<typename Derived>
// Eigen::MatrixXd grad2g(Eigen::MatrixBase<Derived>& x) {
// 	// 设定g梯度的梯度
// 	Eigen::MatrixXd g2g(2, 2);
// 	double x1 = x(0, 0);
// 	double x2 = x(1, 0);
// 	g2g << 0, 0,
// 		0, 0;
// 	return g2g;
// }
// #pragma endregion
 
// void sqp_subp_activeSet() {
// 	// 有效集解子问题 解SQP
// 	int dim = 2;	// 未知数个数
// 	int ne = 1;	// 等式约束的个数
// 	int ni = 1; // 不等式约束的个数
// 	// 设定4个梯度矩阵
// 	Eigen::MatrixXd gf(dim, 1);
// 	Eigen::MatrixXd g2f(dim, dim);
// 	Eigen::MatrixXd gh(dim, 1);
// 	Eigen::MatrixXd g2h(dim, dim);
// 	Eigen::MatrixXd gg(dim, 1);
// 	Eigen::MatrixXd g2g(dim, dim);
// 	//给定初始解
// 	Eigen::MatrixXd x(dim, 1);
// 	x << 2, 1;
// 	// 子问题的矩阵
// 	Eigen::MatrixXd H(dim, dim);
// 	Eigen::MatrixXd c(dim, 1);
// 	Eigen::MatrixXd Ae(ne, dim);
// 	Eigen::MatrixXd be(ne, 1);
// 	// 设定不等式的时候注意大于号
// 	Eigen::MatrixXd Ai(ni, dim);
// 	Eigen::MatrixXd bi(ni, 1);
// 	// 子问题的解
// 	Eigen::MatrixXd d(dim, 1);
// 	int iterTimes = 0;
// 	while (iterTimes < 100) {
// 		c = gradf(x);
// 		H = grad2f(x);
// 		Ae = gradh(x).transpose();
// 		Ai = gradg(x).transpose();
// 		be = -1 * h(x);
// 		bi = -1 * g(x);
// 		d << 0, 0;
// 		qp_activeset(H, c, Ae, be, Ai, bi, ne, ni, d);
// 		x = x + d;
// 	}
// }
 
// #pragma region Smooth Newton Method
// double phi(double& epsilon, double& a, double& b) {
// 	double p = a + b - sqrt(a * a + b * b + 2 * epsilon * epsilon);
// 	return p;
// }
 
// template<typename Derived>
// Eigen::MatrixXd form_H(Eigen::MatrixBase<Derived>& z, int& n, int& l, int& m,
// 	Eigen::MatrixBase<Derived>& Bk, Eigen::MatrixBase<Derived>& df_xk, Eigen::MatrixBase<Derived>& AkE, Eigen::MatrixBase<Derived>& AkI, Eigen::MatrixBase<Derived>& h_xk, Eigen::MatrixBase<Derived>& g_xk) {
// 	double epsilon = z(0, 0);
// 	Eigen::MatrixXd d = z.block(1, 0, n, 1);
// 	Eigen::MatrixXd mu = z.block(1 + n, 0, l, 1);
// 	Eigen::MatrixXd lambda = z.block(1 + n + l, 0, m, 1);
// 	Eigen::MatrixXd H1(d.rows(), 1);
// 	H1 = Bk * d - AkE.transpose() * mu - AkI.transpose() * lambda + df_xk;
// 	Eigen::MatrixXd H2(d.rows(), 1);
// 	H2 = h_xk + AkE * d;
// 	Eigen::MatrixXd Phi(lambda.rows(), 1);
// 	Eigen::MatrixXd b = g_xk + AkI * d;
// 	for (int i = 0; i < lambda.rows(); i++) {
// 		double p = phi(epsilon, lambda(i, 0), b(i, 0));
// 		Phi(i, 0) = p;
// 	}
// 	Eigen::MatrixXd H(1 + H1.rows() + H2.rows() + Phi.rows(), 1);
// 	H(0, 0) = epsilon;
// 	H.block(1, 0, H1.rows(), 1) = H1;
// 	H.block(1 + H1.rows(), 0, H2.rows(), 1) = H2;
// 	H.block(1 + H1.rows() + H2.rows(), 0, Phi.rows(), 1) = Phi;
// 	return H;
// }
 
// template<typename Derived>
// Eigen::MatrixXd JacobH(Eigen::MatrixBase<Derived>& z, int& n, int& l, int& m, Eigen::MatrixBase<Derived>& Bk, Eigen::MatrixBase<Derived>& AkE, Eigen::MatrixBase<Derived>& AkI, Eigen::MatrixBase<Derived>& g_xk) {
// 	Eigen::MatrixXd d = z.block(1, 0, n, 1);
// 	Eigen::MatrixXd mu = z.block(1 + n, 0, l, 1);
// 	Eigen::MatrixXd lambda = z.block(1 + n + l, 0, m, 1);
// 	double epsilon = z(0, 0);
// 	Eigen::MatrixXd temp(lambda.rows(), 1);
// 	temp = g_xk + AkI * d;
// 	Eigen::MatrixXd D1 = Eigen::MatrixXd::Zero(lambda.rows(), lambda.rows());
// 	for (int i = 0; i < lambda.rows(); i++) {
// 		D1(i, i) = 1 - lambda(i, 0) / sqrt(lambda(i, 0) * lambda(i, 0) + temp(i, 0) * temp(i, 0) + 2 * epsilon * epsilon);
// 	}
 
// 	Eigen::MatrixXd D2 = Eigen::MatrixXd::Zero(lambda.rows(), lambda.rows());
// 	for (int i = 0; i < lambda.rows(); i++) {
// 		D2(i, i) = 1 - temp(i, 0) / sqrt(lambda(i, 0) * lambda(i, 0) + temp(i, 0) * temp(i, 0) + 2 * epsilon * epsilon);
// 	}
 
// 	Eigen::MatrixXd nu(lambda.rows(), 1);
// 	for (int i = 0; i < lambda.rows(); i++) {
// 		nu(i, 0) = -2 * epsilon / sqrt(lambda(i, 0) * lambda(i, 0) + temp(i, 0) * temp(i, 0) + 2 * epsilon * epsilon);
// 	}
 
// 	Eigen::MatrixXd dH = Eigen::MatrixXd::Zero(1 + Bk.rows() + AkE.rows() + nu.rows(), 1 + Bk.cols() + AkE.rows() + AkI.rows());
// 	dH.block(0, 0, 1, 1) = Eigen::MatrixXd::Ones(1, 1);
// 	dH.block(1, 1, Bk.rows(), Bk.cols()) = Bk;
// 	dH.block(1, 1 + Bk.cols(), AkE.cols(), AkE.rows()) = -1 * AkE.transpose();
// 	dH.block(1, 1 + Bk.cols() + AkE.rows(), AkI.cols(), AkI.rows()) = -1 * AkI.transpose();
// 	dH.block(1 + Bk.rows(), 1, AkE.rows(), AkE.cols()) = AkE;
// 	dH.block(1 + Bk.rows() + AkE.rows(), 0, nu.rows(), nu.cols()) = nu;
// 	Eigen::MatrixXd D2AkI = D2 * AkI;
// 	dH.block(1 + Bk.rows() + AkE.rows(), nu.cols(), D2AkI.rows(), D2AkI.cols()) = D2AkI;
// 	dH.block(1 + Bk.rows() + AkE.rows(), nu.cols() + D2AkI.cols() + AkE.rows(), D1.rows(), D1.cols()) = D1;
 
// 	return dH;
// }
 
// template<typename Derived>
// Eigen::MatrixXd qp_smooth_Newton_method(Eigen::MatrixBase<Derived>& Bk, Eigen::MatrixBase<Derived>& df_xk, Eigen::MatrixBase<Derived>& AkE, Eigen::MatrixBase<Derived>& AkI,
// 	Eigen::MatrixBase<Derived>& h_xk, Eigen::MatrixBase<Derived>& g_xk,
// 	int& n, int& l, int& m) {
// 	// n-未知数个数，对应d  l-等式约束个数 对应mu h(x)  m-不等式约束个数 对应lambda g(x)
//  	double epsilon = 0.05;
// 	double ep0 = 0.05;
// 	double gamma = 0.05;
// 	double rho = 0.5;
// 	double sigma = 0.2;
// 	Eigen::MatrixXd z = Eigen::MatrixXd::Zero(1 + n + l + m, 1);
// 	z(0, 0) = epsilon;
// 	z.block(1, 0, n, 1) = Eigen::MatrixXd::Ones(n, 1);
// 	int iterTimes = 0;
// 	while (iterTimes < 150) {
// 		Eigen::MatrixXd H = form_H(z, n, l, m, Bk, df_xk, AkE, AkI, h_xk, g_xk);
// 		double norm_H = H.norm();
// 		// 停算判断
// 		if (norm_H <= 1e-6) break;
// 		// 计算 dz
// 		double min_temp = (1 < norm_H) ? 1 : norm_H;
// 		double beta = gamma * norm_H * min_temp;
// 		Eigen::MatrixXd dH = JacobH(z, n, l, m, Bk, AkE, AkI, g_xk);
// 		Eigen::MatrixXd z_bar = Eigen::MatrixXd::Zero(1 + n + l + m, 1);
// 		z_bar(0, 0) = epsilon;
// 		Eigen::MatrixXd dz = dH.inverse() * (beta * z_bar - H);
// 		// 计算步长alpha
// 		int i = 0;
// 		int mk = 0;
// 		while (mk <= 20) {
// 			Eigen::MatrixXd z_ = z + pow(rho, i) * dz;
// 			Eigen::MatrixXd H_rho = form_H(z_, n, l, m, Bk, df_xk, AkE, AkI, h_xk, g_xk);
// 			double norm_H_rho = H_rho.norm();
// 			double rho_temp = (1 - sigma * (1 - gamma * ep0) * pow(rho, i)) * norm_H;
// 			if (norm_H_rho <= rho_temp) {
// 				mk = i;
// 				break;
// 			}
// 			i += 1;
// 			if (i == 20) {
// 				mk = 10;
// 			}
// 		}
 
// 		double alpha = pow(rho, mk);
// 		z = z + alpha * dz;
// 		iterTimes += 1;
// 	}
// 	return z.block(1, 0, n, 1);
// }
 
// #pragma endregion
 
// void sqp_subp_smoothNewtionMethod() {
// 	// 有效集解子问题 解SQP
// 	int dim = 2;	// 未知数个数
// 	int ne = 1;	// 等式约束的个数
// 	int ni = 1; // 不等式约束的个数
// 	// 设定4个梯度矩阵
// 	Eigen::MatrixXd gf(dim, 1);
// 	Eigen::MatrixXd g2f(dim, dim);
// 	Eigen::MatrixXd gh(dim, 1);
// 	Eigen::MatrixXd g2h(dim, dim);
// 	Eigen::MatrixXd gg(dim, 1);
// 	Eigen::MatrixXd g2g(dim, dim);
// 	//给定初始解
// 	Eigen::MatrixXd x(dim, 1);
// 	x << 2, 1;
// 	// 子问题的矩阵
// 	Eigen::MatrixXd H(dim, dim);
// 	Eigen::MatrixXd c(dim, 1);
// 	Eigen::MatrixXd Ae(ne, dim);
// 	Eigen::MatrixXd be(ne, 1);
// 	// 设定不等式的时候注意大于号
// 	Eigen::MatrixXd Ai(ni, dim);
// 	Eigen::MatrixXd bi(ni, 1);
// 	int iterTimes = 0;
// 	while (iterTimes < 100) {
// 		c = gradf(x);
// 		H = grad2f(x);
// 		Ae = gradh(x).transpose();
// 		Ai = gradg(x).transpose();
// 		Eigen::MatrixXd hx = h(x);
// 		Eigen::MatrixXd gx = g(x);
// 		Eigen::MatrixXd d = qp_smooth_Newton_method(H, c, Ae, Ai, hx, gx, dim, ne, ni);
// 		if (d.norm() < 1e-6) break;
// 		x = x + d;
// 	}
// }
 
// // int main()
// // {
// // 	//int n = 2;
// // 	//int l = 0;
// // 	//int m = 5;
// // 	//Eigen::MatrixXd dfk(n, 1);
// // 	//dfk << -6, -2;
// // 	//Eigen::MatrixXd Bk(n, n);
// // 	//Bk << 1, -1,
// // 	//	-1, 2;
// // 	//Eigen::MatrixXd Ae(l, n);
// // 	//Eigen::MatrixXd hk(l, 1);
// // 	//Eigen::MatrixXd Ai(m, n);
// // 	//Ai << -2, -1,
// // 	//	1, -1,
// // 	//	-1, -2,
// // 	//	1, 0,
// // 	//	0, 1;
// // 	//Eigen::MatrixXd gk(m, 1);
// // 	//gk << 3, 1, 2, 0, 0;
 
// // 	//Eigen::MatrixXd d = qp_smooth_Newton_method(Bk, dfk, Ae, Ai, hk, gk, n, l, m);
 
// // 	sqp_subp_smoothNewtionMethod();
 
// // 	system("pause");
// // }