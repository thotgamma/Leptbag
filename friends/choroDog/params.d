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

//身体パーツパラメータ
struct partParam{

	vertexManager vertices;
	vec3 position;
	vec3 scale;
	quat rotation;
	float mass; //総体重に対する百分率
	float friction; //摩擦係数

}



//ヒンジパラメータ
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

//g6dofパラメータ
struct g6dofParam{

	string name;
	bool enabled;
	vec3 position;
	quat rotation;
	string object1Name;
	string object2Name;
	vec3 object1Position;
	vec3 object2Position;
	bool[3] useAngLimit; //(x, y, z) : ( 0, 1, 2 )
	vec3 angLimitLower;
	vec3 angLimitUpper;
	bool[3] useLinLimit;
	vec3 linLimitLower;
	vec3 linLimitUpper;

}

//遺伝させるパラメータ
struct oscillator2Gene{

	vec3[string] angLimitLower;
	vec3[string] angLimitUpper;
	float friction;
	vec3[string] maxForce; //最大出力．いくらmaxVeloを大きくしてもこれ以上の力では駆動しない．
	vec3[string] maxVelo; //g6dofを動かす最高速
	oscillator2 oscil; //振動子モデル．1個体に1つ．
	int degree; //振動子モデルの近似精度(sin(nx), cos(nx)のn)

	//関節間で共通するパラメータの初期化
	void init(){
		degree = uniform(1, 10, rnd);
		oscil = new oscillator2(degree);
		friction = uniform(0.0f, 5.0f, rnd);
	}


	//各関節で異なるパラメータの初期化
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

	//表示関数．oscilのtoString()は実装してない．
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
