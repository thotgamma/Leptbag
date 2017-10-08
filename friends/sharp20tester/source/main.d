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

extern (C) void init() {
	rt_init();
	try {

	} catch (Exception ex){
		writeln(ex.toString);
	}
}

extern (C) void tick() {
	try {

	} catch (Exception ex){
		writeln(ex.toString);
	}
}



//------------------------------------------
