import core.stdc.stdio;
import std.stdio;
import core.runtime;
import std.random;
import std.math;
import std.algorithm;
import std.conv;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

//ApplicationInterface----------------------

elementNode block;
int timer;

extern (C) void init() {
	rt_init();
	try {
		block = getCubeShape().generate(parameterPack(
			param("position", Vector3f( 0, 10, 0)),
			param("scale",    Vector3f(  1, 1, 1)),
			param("rotation", Quaternionf(0, 0, 0, 1)),
			param("mass", 1.0f)));


	} catch (Exception ex){
		writeln(ex.toString);
	}
}


extern (C) void tick() {
	try {

		if (timer == 10) {
			block.destroy();

			block = getCubeShape().generate(parameterPack(
				param("position", Vector3f( 0, 10, 0)),
				param("scale",    Vector3f(  1, 1, 1)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 1.0f)));

			timer = 0;
		} else {
			timer ++;
		}


	} catch (Exception ex){
		writeln(ex.toString);
	}
}



//------------------------------------------
