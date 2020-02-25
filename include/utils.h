#pragma once
#include <Eigen/Dense>
#include <vector>
#include <iostream>

#define VERBOSE true

#ifndef LOG_ERROR
#define LOG_ERROR(X) ::utils::log_error(X, __FILE__, __LINE__, __func__)
#endif 

#ifndef LOG_INFO
#define LOG_INFO(...) ::utils::log_info(__VA_ARGS__)
#endif 

#ifndef LOG_ERROR_IF
#define LOG_ERROR_IF(Pred, X) if(Pred) LOG_ERROR(X)
#endif 

namespace utils {
    static bool verbose = VERBOSE;

	static void log_error(std::string e, const char* file, int line, const char* func) {
		std::cout << "ERROR: \"" << e << "\" in file " << file << "::" << line << " " << func << "(...)" << std::endl;
	}

    template<typename ... Args>
    static void log_info(const std::string& format, Args ... args) {
        if (verbose) {
            std::printf("Info(): ");
            std::printf(format.c_str(), args ...);
            std::printf("\n");
        }
    }

	//TODO: Matrix per Reference and not value!!
	template<typename _Scalar>
	static Eigen::Matrix<_Scalar, -1, -1> vector2d_to_eigenmatrix(const std::vector<std::vector<_Scalar>> vector2d, int rows, int cols) {
		Eigen::Matrix<_Scalar, -1, -1> mat(rows, cols);

		if (vector2d.size() < rows) {
			LOG_ERROR("Dimension Mismatch");
			return mat;
		}

		for (int i = 0; i < cols; ++i) {
			for (int j = 0; j < rows; ++j)
				mat(j,i) = vector2d[j][i];
		}
		return mat;
	}

	template<typename _Scalar>
	static Eigen::Matrix<_Scalar, -1, -1> get_rows_from_idx_vec(const Eigen::Matrix<_Scalar, -1, -1>& values, const std::vector<int>& idx_vec) {
        Eigen::MatrixXf new_values(idx_vec.size(), values.cols());
		for (int i = 0; i < idx_vec.size(); ++i)
			new_values.row(i) = values.row(idx_vec[i]);
		return new_values;
	}

	//TODO: Not working
	template<typename _Scalar>
	static void softmax_col(Eigen::Matrix<_Scalar, -1, -1>& m) {
        for (int i = 0; i < m.cols(); ++i) {
            m.col(i) = m.col(i).unaryExpr([](float c) { return std::exp(c); });
            float sum = m.col(i).sum();
            m.col(i) = m.col(i).unaryExpr([sum](float c) { return c / sum; });
        }
	}

    template <typename T1, typename T2>
    void remove_idx_over_criteria_in_second_vec(std::vector<T1>& v, std::vector<T2>& second, T2 criteria) {
        LOG_ERROR_IF(second.size() != v.size(), "\"second\" size must match v size");
        v.erase(std::remove_if(std::begin(v), std::end(v), [&](T1& elem) {
                    int idx = &elem - &v[0];
                    if (second[idx] > criteria)
                        return true;
                    return false;
                }),
            std::end(v));
	    }
}