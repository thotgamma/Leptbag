import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;


struct agentBodyParameter{
	elementManager[string] partsGenerator;
	partParam[string] partParams; //身体パーツのパラメータ
	g6dofParam[string] g6dofParams; //g6dofのパラメータ
}

//身体パーツパラメータ
struct partParam{

	vertexManager vertices;
	Vector3f position;
	Vector3f scale;
	Quaternionf rotation;
	float mass; //総体重に対する百分率
	float friction; //摩擦係数

}



//ヒンジパラメータ
struct hingeParam{

	string name;
	Vector3f position;
	Vector3f axis1;
	Vector3f axis2;
	string object1Name;
	string object2Name;
	Vector3f object1Position;
	Vector3f object2Position;
	bool enabled;
	bool useLimit;
	float limitLower;
	float limitUpper;


}

//g6dofパラメータ
struct g6dofParam{

	string name;
	bool enabled;
	Vector3f position;
	Quaternionf rotation;
	string object1Name;
	string object2Name;
	Vector3f object1Position;
	Vector3f object2Position;
	bool[3] useAngLimit; //(x, y, z) : ( 0, 1, 2 )
	Vector3f angLimitLower;
	Vector3f angLimitUpper;
	bool[3] useLinLimit;
	Vector3f linLimitLower;
	Vector3f linLimitUpper;

}


//遺伝させるパラメータ
struct serialOrderGene{

	static uint lengthOfSet = 6;
	Vector3f[string][] tracks;
	bool[][] wavelengthOfOrder;
	int[] moveSpan;
	float friction;
	float maxRotationalMotorForce;
	Vector3f[string][] maxVelocity;

	void init(){

		auto rnd = Random(unpredictableSeed);

		tracks.length = lengthOfSet;
		moveSpan.length = lengthOfSet;
		maxVelocity.length = lengthOfSet;

		friction = 1.0f;//uniform(0.1f, 8.0f, rnd);
		maxRotationalMotorForce = 10.0f;//uniform(0.0f, 30.0f, rnd);

		for(int i=0; i<moveSpan.length; i++) moveSpan[i] = uniform(1, 10, rnd);

		wavelengthOfOrder.length = lengthOfSet;
		for(int i=0; i<wavelengthOfOrder.length; i++){

			wavelengthOfOrder[i].length = moveSpan[i];

			for(int j=0; j<wavelengthOfOrder[i].length; j++){
				if(uniform(0.0f, 1.0f, rnd) < 0.5f) wavelengthOfOrder[i][j] = true;
				else wavelengthOfOrder[i][j] = false;
			}

		}

	}


	void init(string s, Vector3f lowerLimit, Vector3f upperLimit){

		auto rnd = Random(unpredictableSeed);

		for(int i=0; i<lengthOfSet; i++){

			float x, y, z;
			if(lowerLimit.x<upperLimit.x){
				x = uniform(lowerLimit.x, upperLimit.x, rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.y<upperLimit.y){
				y = uniform(lowerLimit.y, upperLimit.y, rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.z<upperLimit.z){
				z = uniform(lowerLimit.z, upperLimit.z, rnd);
			}else{
				z = 0.0f;
			}

			tracks[i][s] = Vector3f(x, y, z);
		}

		for(int i=0; i<lengthOfSet; i++){

			float x, y, z;
			if(lowerLimit.x<upperLimit.x){
				x = uniform(0.0f, 20.0f, rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.y<upperLimit.y){
				y = uniform(0.0f, 20.0f, rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.z<upperLimit.z){
				z = uniform(0.0f, 20.0f, rnd);
			}else{
				z = 0.0f;
			}

			//maxVelocity[i][s] = Vector3f(x, y, z);
			maxVelocity[i][s] = Vector3f(5.0f, 0.0f, 0.0f);
		}

	}

	void init(int i, string s, Vector3f lowerLimit, Vector3f upperLimit){
		auto rnd = Random(unpredictableSeed);

			float x, y, z;
			if(lowerLimit.x<upperLimit.x){
				x = uniform(lowerLimit.x, upperLimit.x, rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.y<upperLimit.y){
				y = uniform(lowerLimit.y, upperLimit.y, rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.z<upperLimit.z){
				z = uniform(lowerLimit.z, upperLimit.z, rnd);
			}else{
				z = 0.0f;
			}

			tracks[i][s] = Vector3f(x, y, z);

	}


	void copytracks(serialOrderGene u){
		foreach(int i, elem1; this.tracks){
			foreach(string s, elem2; elem1){
				this.tracks[i][s] = u.tracks[i][s];
			}
		}
	}

	void copytracks(serialOrderGene u,int i,string s){
		this.tracks[i][s] = u.tracks[i][s];
	}


}



