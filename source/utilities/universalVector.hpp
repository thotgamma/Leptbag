#ifndef UNIVERSAL_VECTOR_HPP 
#define UNIVERSAL_VECTOR_HPP

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <bullet/btBulletDynamicsCommon.h>


class vec3{

	public:
	static int count;
	virtual float getx();
	virtual float gety();
	virtual float getz();

	float x;
	float y;
	float z;

	vec3();
	vec3(float x, float y, float z);
	vec3(const vec3& rhs);
	virtual ~vec3();

	btVector3 toBullet();
	glm::vec3 toGlm();

	vec3& operator= (const vec3& R){
		this->x = R.x;
		this->y = R.y;
		this->z = R.z;
		return *this;
	}
};

extern "C" vec3* createVec3(float x, float y, float z);

class quat{

	public:
	static int count;
	float w;
	float x;
	float y;
	float z;

	quat();
	quat(float w, float x, float y, float z);
	quat(const quat& rhs);
	virtual ~quat();
	virtual float getw();
	virtual float getx();
	virtual float gety();
	virtual float getz();
	btQuaternion toBullet();
	glm::quat toGlm();
};


extern "C" quat* createQuat(float w, float x, float y, float z);


#endif
