#include "universalVector.hpp"

vec3::vec3(){
	x = y = z = 0;
}

vec3::vec3(float x, float y, float z){
	this->x = x;
	this->y = y;
	this->z = z;
}

btVector3 vec3::toBullet(){
	return btVector3(x, y, z);
}

glm::vec3 vec3::toGlm(){
	return glm::vec3(x, y, z);
}


quat::quarternion(){
	w = 1;
	x = y = z = 0;
}

quat::quarternion(float w, float x, float y, float z){
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}


btQuaternion quat::toBullet(){
	return btQuaternion(w, x, y, z);
}

glm::quat quat::toGlm(){
	return glm::quat(x, y, z, w);
}