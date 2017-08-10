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
		degree = 5;//uniform(1, 10, rnd);
		oscil = new oscillator2(degree);
		friction = 0.8;//uniform(0.0f, 5.0f, rnd);
	}


	void init(string s){

		maxForce[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), 0.0f );
		maxVelo[s] = createVec3( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd) );

		oscil.init(s);

		angLimitUpper[s] = createVec3( uniform(0.0f, 1.57f, rnd), uniform(0.0f, 1.57f, rnd), 0.0f );
		angLimitLower[s] = createVec3( uniform(-1.57f, 0.0f, rnd), uniform(-1.57f, 0.0f, rnd), 0.0f );

	}

	void rehash(){
		angLimitLower.rehash;
		angLimitUpper.rehash;
		maxForce.rehash;
		maxVelo.rehash;
	}

	void toString(){

		write("angLimitLower : ");
		foreach(string s, elem; angLimitLower){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		write("angLimitUpper : ");
		foreach(string s, elem; angLimitUpper){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		writeln("friction = ", friction);

		write("maxForce : ");
		foreach(string s, elem; maxForce){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		write("maxVelo : ");
		foreach(string s, elem; maxVelo){
			write("[ \"", s, "\": ", elem.getx(), ", ", elem.gety(), ", ", elem.getz(), " ], ");
		}
		writeln("");

		//oscil.toString();
		writeln("degree = ", degree);


	}


}
