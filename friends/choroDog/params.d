import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import GA;
import Oscillator;

struct partParam{

	vertexManager vertices;
	vec3 position;
	vec3 scale;
	quat rotation;
	float mass;
	float friction;

}



struct hingeParam{

	string name;
	vec3 position;
	vec3 axis1;
	vec3 axis2;
	string object1Name;
	string object2Name;
	vec3 object1Position;
	vec3 object2Position;
	bool enabled;
	bool useLimit;
	float limitLower;
	float limitUpper;


}

struct g6dofParam{

	string name;
	bool enabled;
	vec3 position;
	quat rotation;
	string object1Name;
	string object2Name;
	vec3 object1Position;
	vec3 object2Position;
	bool[3] useAngLimit;
	vec3 angLimitLower;
	vec3 angLimitUpper;
	bool[3] useLinLimit;
	vec3 linLimitLower;
	vec3 linLimitUpper;

}

struct oscillator2Gene{

	vec3[string] angLimitLower;
	vec3[string] angLimitUpper;
	float friction;
	vec3[string] maxForce;
	vec3[string] maxVelo;
	oscillator2 oscil;
	int degree;

	void init(){
		degree = uniform(1, 10, rnd);
		oscil = new oscillator2(degree);
		friction = uniform(0.0f, 5.0f, rnd);
	}


	void init(string s){

		maxForce[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd) );
		maxVelo[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd) );

		oscil.init(s);

		angLimitUpper[s] = createVec3( uniform(0.0f, 1.57f/2.0f, rnd), uniform(0.0f, 1.57f/2.0f, rnd), 0.0f );
		angLimitLower[s] = createVec3( uniform(0.0f, 1.57f/2.0f, rnd), uniform(0.0f, 1.57f/2.0f, rnd), 0.0f );

	}

	void rehash(){
		angLimitLower.rehash;
		angLimitUpper.rehash;
		maxForce.rehash;
		maxVelo.rehash;
	}

	/+
	void rehash(){
		angLimit.rehash;
		maxForce.rehash;
		maxVelo.rehash;
	}
	+/

}
