#include "Movable.h"

Movable::Movable()
{
	reset();
}

void Movable::reset()
{
	rotations = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	translate = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	center = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	//translateSystem = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	rotate = Eigen::Matrix4f::Identity();
}


Eigen::Matrix4f Movable::MakeTrans()
{
	return translate.matrix() * center.inverse().matrix() * rotate * rotations.matrix() * center.matrix();//* translateSystem.matrix();
}

Eigen::Matrix4f Movable::getRotateMatrix()
{
	return	rotate * rotations.matrix();
}
void Movable::MyTranslate(Eigen::Vector3f amt)
{
	translate.pretranslate(amt);
}
//void Movable::MyTranslateInSystem(Eigen::Vector3f amt)
//{
//	translateSystem.pretranslate(amt);
//	
//}
//angle in radians
void Movable::RotateLink(Eigen::Vector3f rotAxis, float angle)
{
	if (rotAxis(1) == 1)
	{
		rotAxis = rotate.inverse().block<3, 3>(0, 0) * rotAxis;// * rotAxis;
	}
	rotations.rotate(Eigen::AngleAxisf(angle, rotAxis));
	Eigen::Matrix4f temp;
	temp = rotate * rotations.matrix();
	rotations = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	rotate = temp;

}

//void Movable::RotateInSystem( Eigen::Vector3f rotAxis, float angle)
//{
//	if(rotAxis(1) == 1)
//		rotAxis =rotate.inverse().block<3,3>(0,0)* rotAxis;// * rotAxis;
//	rotations.rotate(Eigen::AngleAxisf(angle, rotAxis));
//
//	Eigen::Matrix4f temp;
//	temp = rotations.matrix()*rotate ;
//	rotations = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
//	rotate = temp;
//
//	
//}



void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	rotations.rotate(Eigen::AngleAxisf(angle, rotAxis));
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	translate.scale(amt);
}

void Movable::rotateForIK(Eigen::Matrix3f& mat)
{
	Eigen::Matrix4f newMatr;
	newMatr << mat(0), mat(1), mat(2), 0,
		mat(3), mat(4), mat(5), 0,
		mat(6), mat(7), mat(8), 0,
		0, 0, 0, 1;
	Eigen::Matrix4f temp;
	temp = rotate * rotations.matrix() * newMatr;
	rotations = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	rotate = temp;
}

void Movable::SetCenterRotation(Eigen::Vector3f amt)
{
	center.translate(amt);
}

