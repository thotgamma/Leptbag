import std.stdio;
import std.json;
import std.conv;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

import params;


const string loadmodel = "models/oitake.fpm";



void loadMesh(ref partParam[string] partParams){

	try{
		auto jsonString = import(loadmodel);

		JSONValue model = parseJSON(jsonString);

		foreach(elem; model.array){
			if(elem["objectType"].str == "MESH"){

				string name = elem["name"].str;

				partParams[name] = partParam();
				partParams[name].position = Vector3f(elem["xpos"].floating, elem["ypos"].floating, elem["zpos"].floating);
				partParams[name].scale = Vector3f(elem["xscl"].floating, elem["yscl"].floating, elem["zscl"].floating);
				partParams[name].rotation = Quaternionf(elem["xqat"].floating, elem["yqat"].floating, elem["zqat"].floating, elem["wqat"].floating);
				auto existMass = "mass" in elem;
				if(existMass!=null){
					partParams[name].mass = elem["mass"].floating;
				}else{
					partParams[name].mass = 0.0f;
				}

				auto existFriction = "friction" in elem;
				if(existFriction!=null){
					partParams[name].friction = elem["friction"].floating;
				}else{
					partParams[name].friction = 0.0f;
				}

				partParams[name].vertices = new vertexManager();

				foreach(objvertex; elem["vertex"].array){
					partParams[name].vertices.addVertex(
							objvertex.array[0].floating, objvertex.array[1].floating, objvertex.array[2].floating,
							objvertex.array[3].floating, objvertex.array[4].floating, objvertex.array[5].floating,
							objvertex.array[6].floating, objvertex.array[7].floating, objvertex.array[8].floating );
					/+
				writeln(	name, " : ", objvertex.array[0].floating, ", ",  objvertex.array[1].floating, ", ",  objvertex.array[2].floating, ", ",
							objvertex.array[3].floating, ", ",  objvertex.array[4].floating, ", ",  objvertex.array[5].floating, ", ", 
							objvertex.array[6].floating, ", ",  objvertex.array[7].floating, ", ",  objvertex.array[8].floating );
				+/
				}

			}

		}
	}catch(Exception ex){
		writeln(ex.toString);
	}

}


//テストしてない
void loadHinge(ref hingeParam[string] hingeParams){

	try{
		//HACK コンパイル時にjsonStringにlowPolyTree.fpmの内容が代入される(要-Jオプション)
		auto jsonString = import(loadmodel);

		JSONValue model = parseJSON(jsonString);

		foreach(elem; model.array){
			if(elem["objectType"].str == "hingeConstraint"){

				string name = elem["name"].str;
				hingeParams[name] = hingeParam();
				hingeParams[name].name = name;

				hingeParams[name].axis1 = Vector3f(elem["xaxs1"].floating , elem["yaxs1"].floating, elem["zaxs1"].floating);
				hingeParams[name].axis2 = Vector3f(elem["xaxs2"].floating , elem["yaxs2"].floating, elem["zaxs2"].floating);


				if(elem["enabled"].str == "True") hingeParams[name].enabled = true; else hingeParams[name].enabled = false;
				if(elem["useLimit"].str == "True") hingeParams[name].useLimit = true; else hingeParams[name].useLimit = false;
				hingeParams[name].limitLower = elem["limitLower"].floating;
				hingeParams[name].limitUpper = elem["limitUpper"].floating;
				hingeParams[name].object1Position = Vector3f(elem["object1xpos"].floating, elem["object1ypos"].floating, elem["object1zpos"].floating);
				hingeParams[name].object2Position = Vector3f(elem["object2xpos"].floating, elem["object2ypos"].floating, elem["object2zpos"].floating);
				hingeParams[name].object1Name = elem["object1"].str;
				hingeParams[name].object2Name = elem["object2"].str;


			}
		}
	}catch(Exception ex){
		writeln(ex.toString);
	}

}


void loadG6dof(ref g6dofParam[string] g6dofParams){

	try{
		//HACK コンパイル時にjsonStringにlowPolyTree.fpmの内容が代入される(要-Jオプション)
		auto jsonString = import(loadmodel);

		JSONValue model = parseJSON(jsonString);

		foreach(elem; model.array){
			if(elem["objectType"].str == "genericConstraint"){

				string name = elem["name"].str;
				g6dofParams[name] = g6dofParam();
				g6dofParams[name].name = name;

				if(elem["enabled"].str == "True") g6dofParams[name].enabled = true;
				else g6dofParams[name].enabled = false;

				g6dofParams[name].position = Vector3f(to!float(elem["xpos"].str), to!float(elem["ypos"].str), to!float(elem["zpos"].str));
				g6dofParams[name].rotation = Quaternionf(elem["xqat"].floating, elem["yqat"].floating, elem["zqat"].floating, elem["wqat"].floating);

				g6dofParams[name].object1Name = elem["object1"].str;
				g6dofParams[name].object2Name = elem["object2"].str;

				g6dofParams[name].object1Position = Vector3f(elem["object1xpos"].floating, elem["object1ypos"].floating, elem["object1zpos"].floating);
				g6dofParams[name].object2Position = Vector3f(elem["object2xpos"].floating, elem["object2ypos"].floating, elem["object2zpos"].floating);

				if(elem["useXAngLimit"].str == "True") g6dofParams[name].useAngLimit[0]= true; else g6dofParams[name].useAngLimit[0] = false;
				if(elem["useYAngLimit"].str == "True") g6dofParams[name].useAngLimit[1]= true; else g6dofParams[name].useAngLimit[1] = false;
				if(elem["useZAngLimit"].str == "True") g6dofParams[name].useAngLimit[2]= true; else g6dofParams[name].useAngLimit[2] = false;

				g6dofParams[name].angLimitLower = Vector3f(elem["xAngLimitLower"].floating, elem["yAngLimitLower"].floating, elem["zAngLimitLower"].floating);
				g6dofParams[name].angLimitUpper = Vector3f(elem["xAngLimitUpper"].floating, elem["yAngLimitUpper"].floating, elem["zAngLimitUpper"].floating);

				if(elem["useXLinLimit"].str == "True") g6dofParams[name].useLinLimit[0]= true; else g6dofParams[name].useLinLimit[0] = false;
				if(elem["useYAngLimit"].str == "True") g6dofParams[name].useLinLimit[1]= true; else g6dofParams[name].useLinLimit[1] = false;
				if(elem["useZAngLimit"].str == "True") g6dofParams[name].useLinLimit[2]= true; else g6dofParams[name].useLinLimit[2] = false;

				g6dofParams[name].linLimitLower = Vector3f(elem["xLinLimitLower"].floating, elem["yLinLimitLower"].floating, elem["zLinLimitLower"].floating);
				g6dofParams[name].linLimitUpper = Vector3f(elem["xLinLimitUpper"].floating, elem["yLinLimitUpper"].floating, elem["zLinLimitUpper"].floating);


			}
		}
	}catch(Exception ex){
		writeln(ex.toString);
	}
}

