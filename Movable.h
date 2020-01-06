#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	void reset();
	Eigen::Matrix4f MakeTrans();
	void MyTranslate(Eigen::Vector3f amt);
	//void MyTranslateInSystem(Eigen::Vector3f amt);
	void MyRotate(Eigen::Vector3f rotAxis, float angle);
	void MyScale(Eigen::Vector3f amt);
	void rotateForIK(Eigen::Matrix3f& mat);
	void SetCenterRotation(Eigen::Vector3f amt);
	void RotateLink(Eigen::Vector3f rotAxis, float angle);
	Eigen::Matrix4f getRotateMatrix();

	//void RotateInSystem(Eigen::Vector3f rotAxis, float angle);

private:
	Eigen::Transform<float, 3, Eigen::Affine> translate;
	Eigen::Transform<float, 3, Eigen::Affine> rotations;
	Eigen::Transform<float, 3, Eigen::Affine> center;
	//Eigen::Transform<float, 3, Eigen::Affine> translateSystem;
	Eigen::Matrix4f rotate;
};

