#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

using Eigen::MatrixXf; //dynamic float Matrix
using Eigen::VectorXi; //dynamic int Vector
using Eigen::VectorXf; 
using std::vector;

struct BoxLabel {
	float score;
	int label_int;
	std::string label_string;
	float coordinates[4];
};

struct DefaultBoxSetting {
	int img_size;
	VectorXi feat_size;
	VectorXi steps;
	VectorXi scales;
	vector<vector<int>> aspect_ratios;
	float scale_xy = 0.1f;
	float scale_wh = 0.2f;
};

class Decoder
{

public:
	bool init_default_boxes300();
	bool init_default_boxes(const DefaultBoxSetting& s);
	vector<BoxLabel> listdecode_batch(MatrixXf& boxes_in, MatrixXf& scores_in, float criteria = 0.45, float max_output = 200);
    

private:
	void scale_back_batch(MatrixXf& bboxes_in, MatrixXf& scores_in);
    vector<float> calc_iou_tensor(MatrixXf& box1, MatrixXf& box2);
	
private:
	bool mInit = false;
	DefaultBoxSetting mSettings;
	MatrixXf mDBoxes;
	MatrixXf mDBoxes_ltrb;

private:
    friend class Test;
};
 
 