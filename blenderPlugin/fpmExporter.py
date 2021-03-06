import numpy as np
import bpy
from bpy_extras.io_utils import ExportHelper

bl_info = {
    "name": "Friends Physical Model Exporter",
    "category": "Import-Export",
    }

def mulQuat(w1, v1, w2, v2):
    w = w1*w2 - np.dot(v1, v2)
    v = w1*v2 + w2*v1 + np.cross(v1, v2)
    return np.array([w, v[0], v[1], v[2]], dtype=float)

def quaternion( position, quat ):


    Quat = quat[1:4]
    QuatW = quat[0]

    QuatInv = -1.0*Quat
    QuatInvW = QuatW

    #上記クォータニオンを適用
    tmp = mulQuat(QuatInvW, QuatInv, 0, position)
    ans = mulQuat(tmp[0], tmp[1:4], QuatW, Quat)

    return ans[1:4]

def getQuatInv( quat ):

    quat[1:4] = -1.0 * quat[1:4]
    return quat

def blenderToBullet( vec3 ):
    return np.array([ vec3[0], vec3[2], -1.0*vec3[1] ])




class fpmExporter(bpy.types.Operator, ExportHelper):
    bl_label = 'export fpm'
    bl_idname = 'export.fpm'
    filename_ext = '.fpm' # ←必須

    def execute(self, context):

        fo = open(self.filepath, 'w')


        fo.write("[\n")

        for keyname in bpy.data.objects.keys():
            obj = bpy.data.objects[keyname]
            if obj.type == "EMPTY":


                if obj.rigid_body_constraint.type == "HINGE":

                        hingeLocation = np.array([obj.location[0], obj.location[1], obj.location[2]], dtype=float)
                        hingeQuat = np.array([obj.rotation_quaternion[0], obj.rotation_quaternion[1], obj.rotation_quaternion[2], obj.rotation_quaternion[3]], dtype=float)
                        hingeQuatInv = getQuatInv(hingeQuat)

                        #ヒンジの相対角度は回転前のobj1, obj2に対するものなので，それを補正
                        obj1Location = np.array([obj.rigid_body_constraint.object1.location[0],
                            obj.rigid_body_constraint.object1.location[1],
                            obj.rigid_body_constraint.object1.location[2]],
                            dtype=float)
                        #クォータニオン
                        obj1Quat = np.array([obj.rigid_body_constraint.object1.rotation_quaternion[0],
                            obj.rigid_body_constraint.object1.rotation_quaternion[1],
                            obj.rigid_body_constraint.object1.rotation_quaternion[2],
                            obj.rigid_body_constraint.object1.rotation_quaternion[3]],
                            dtype=float)

                        #ヒンジの相対角度は回転前のobj2, obj2に対するものなので，それを補正
                        obj2Location = np.array([obj.rigid_body_constraint.object2.location[0],
                            obj.rigid_body_constraint.object2.location[1],
                            obj.rigid_body_constraint.object2.location[2]],
                            dtype=float)
                        #クォータニオン
                        obj2Quat = np.array([obj.rigid_body_constraint.object2.rotation_quaternion[0],
                            obj.rigid_body_constraint.object2.rotation_quaternion[1],
                            obj.rigid_body_constraint.object2.rotation_quaternion[2],
                            obj.rigid_body_constraint.object2.rotation_quaternion[3]],
                            dtype=float)

                        obj1Location = hingeLocation - obj1Location
                        obj1Location = quaternion(obj1Location, obj1Quat)

                        obj2Location = hingeLocation - obj2Location
                        obj2Location = quaternion(obj2Location, obj2Quat)

                        hingeObj1Axis = np.array([ 0.0, 0.0, 1.0 ], dtype=float)
                        hingeObj2Axis = np.array([ 0.0, 0.0, 1.0 ], dtype=float)

                        hingeObj1Axis = quaternion(hingeObj1Axis, hingeQuatInv)
                        hingeObj1Axis = quaternion(hingeObj1Axis, obj1Quat)

                        hingeObj2Axis = quaternion(hingeObj2Axis, hingeQuatInv)
                        hingeObj2Axis = quaternion(hingeObj2Axis, obj2Quat)


                        fo.write("\t{\n")
                        fo.write("\t\t\"objectType\":\"hingeConstraint\",\n")
                        fo.write("\t\t\"name\":\"%s\",\n" % keyname)
                        fo.write("\t\t\"constraintType\":\"%s\",\n" % obj.rigid_body_constraint.type)
                        fo.write("\t\t\"enabled\":\"%s\",\n" % obj.rigid_body_constraint.enabled)

                        fo.write("\t\t\"object1\":\"%s\",\n" % obj.rigid_body_constraint.object1.name)
                        fo.write("\t\t\"object1xpos\":%f,\n" % obj1Location[1])
                        fo.write("\t\t\"object1ypos\":%f,\n" % obj1Location[2])
                        fo.write("\t\t\"object1zpos\":%f,\n" % obj1Location[0])

                        fo.write("\t\t\"object2\":\"%s\",\n" % obj.rigid_body_constraint.object2.name)
                        fo.write("\t\t\"object2xpos\":%f,\n" % obj2Location[1])
                        fo.write("\t\t\"object2ypos\":%f,\n" % obj2Location[2])
                        fo.write("\t\t\"object2zpos\":%f,\n" % obj2Location[0])

                        fo.write("\t\t\"xaxs1\":%f,\n" % hingeObj1Axis[1])
                        fo.write("\t\t\"yaxs1\":%f,\n" % hingeObj1Axis[2])
                        fo.write("\t\t\"zaxs1\":%f,\n" % hingeObj1Axis[0])

                        fo.write("\t\t\"xaxs2\":%f,\n" % hingeObj2Axis[1])
                        fo.write("\t\t\"yaxs2\":%f,\n" % hingeObj2Axis[2])
                        fo.write("\t\t\"zaxs2\":%f,\n" % hingeObj2Axis[0])

                        fo.write("\t\t\"wqat\":%f,\n" % hingeQuat[0])
                        fo.write("\t\t\"xqat\":%f,\n" % hingeQuat[1])
                        fo.write("\t\t\"yqat\":%f,\n" % hingeQuat[3])
                        fo.write("\t\t\"zqat\":%f,\n" % (-1.0*hingeQuat[2]))

                        fo.write("\t\t\"useLimit\":\"%s\",\n" % obj.rigid_body_constraint.use_limit_ang_z)
                        fo.write("\t\t\"limitLower\":%f,\n" % obj.rigid_body_constraint.limit_ang_z_lower)
                        fo.write("\t\t\"limitUpper\":%f\n" % obj.rigid_body_constraint.limit_ang_z_upper)
                        fo.write("\t},\n")




                elif obj.rigid_body_constraint.type == "GENERIC":



                        fo.write("\t{\n")
                        fo.write("\t\t\"objectType\":\"genericConstraint\",\n")
                        fo.write("\t\t\"name\":\"%s\",\n" % keyname)
                        fo.write("\t\t\"constraintType\":\"%s\",\n" % obj.rigid_body_constraint.type)
                        fo.write("\t\t\"enabled\":\"%s\",\n" % obj.rigid_body_constraint.enabled)

                        fo.write("\t\t\"xpos\":\"%s\",\n" % obj.location[1])
                        fo.write("\t\t\"ypos\":\"%s\",\n" % obj.location[2])
                        fo.write("\t\t\"zpos\":\"%s\",\n" % obj.location[0])

                        fo.write("\t\t\"wqat\":%f,\n" % obj.rotation_quaternion[0])
                        fo.write("\t\t\"xqat\":%f,\n" % obj.rotation_quaternion[2])
                        fo.write("\t\t\"yqat\":%f,\n" % obj.rotation_quaternion[3])
                        fo.write("\t\t\"zqat\":%f,\n" % obj.rotation_quaternion[1])

                        fo.write("\t\t\"object1\":\"%s\",\n" % obj.rigid_body_constraint.object1.name)
                        fo.write("\t\t\"object1xpos\":%f,\n" % (obj.location[1]-obj.rigid_body_constraint.object1.location[1]))
                        fo.write("\t\t\"object1ypos\":%f,\n" % (obj.location[2]-obj.rigid_body_constraint.object1.location[2]))
                        fo.write("\t\t\"object1zpos\":%f,\n" % (obj.location[0]-obj.rigid_body_constraint.object1.location[0]))

                        fo.write("\t\t\"object2\":\"%s\",\n" % obj.rigid_body_constraint.object2.name)
                        fo.write("\t\t\"object2xpos\":%f,\n" % (obj.location[1]-obj.rigid_body_constraint.object2.location[1]))
                        fo.write("\t\t\"object2ypos\":%f,\n" % (obj.location[2]-obj.rigid_body_constraint.object2.location[2]))
                        fo.write("\t\t\"object2zpos\":%f,\n" % (obj.location[0]-obj.rigid_body_constraint.object2.location[0]))


                        fo.write("\t\t\"useXAngLimit\":\"%s\",\n" % obj.rigid_body_constraint.use_limit_ang_y)
                        fo.write("\t\t\"useYAngLimit\":\"%s\",\n" % obj.rigid_body_constraint.use_limit_ang_z)
                        fo.write("\t\t\"useZAngLimit\":\"%s\",\n" % obj.rigid_body_constraint.use_limit_ang_x)
                        fo.write("\t\t\"xAngLimitLower\":%f,\n" % obj.rigid_body_constraint.limit_ang_y_lower)
                        fo.write("\t\t\"yAngLimitLower\":%f,\n" % obj.rigid_body_constraint.limit_ang_z_lower)
                        fo.write("\t\t\"zAngLimitLower\":%f,\n" % obj.rigid_body_constraint.limit_ang_x_lower)
                        fo.write("\t\t\"xAngLimitUpper\":%f,\n" % obj.rigid_body_constraint.limit_ang_y_upper)
                        fo.write("\t\t\"yAngLimitUpper\":%f,\n" % obj.rigid_body_constraint.limit_ang_z_upper)
                        fo.write("\t\t\"zAngLimitUpper\":%f,\n" % obj.rigid_body_constraint.limit_ang_x_upper)

                        fo.write("\t\t\"useXLinLimit\":\"%s\",\n" % obj.rigid_body_constraint.use_limit_ang_y)
                        fo.write("\t\t\"useYLinLimit\":\"%s\",\n" % obj.rigid_body_constraint.use_limit_ang_z)
                        fo.write("\t\t\"useZLinLimit\":\"%s\",\n" % obj.rigid_body_constraint.use_limit_ang_x)
                        fo.write("\t\t\"xLinLimitLower\":%f,\n" % obj.rigid_body_constraint.limit_lin_y_lower)
                        fo.write("\t\t\"yLinLimitLower\":%f,\n" % obj.rigid_body_constraint.limit_lin_z_lower)
                        fo.write("\t\t\"zLinLimitLower\":%f,\n" % obj.rigid_body_constraint.limit_lin_x_lower)
                        fo.write("\t\t\"xLinLimitUpper\":%f,\n" % obj.rigid_body_constraint.limit_lin_y_upper)
                        fo.write("\t\t\"yLinLimitUpper\":%f,\n" % obj.rigid_body_constraint.limit_lin_z_upper)
                        fo.write("\t\t\"zLinLimitUpper\":%f\n" % obj.rigid_body_constraint.limit_lin_x_upper)
                        fo.write("\t},\n")






            elif obj.type == "MESH":

                if obj.rigid_body:
                    fo.write("\t{\n")
                    fo.write("\t\t\"objectType\":\"MESH\",\n")
                    fo.write("\t\t\"name\":\"%s\",\n" % keyname)
                    fo.write("\t\t\"rigidBodyType\":\"%s\",\n" % obj.rigid_body.type)
                    fo.write("\t\t\"rigidBodyShape\":\"%s\",\n" % obj.rigid_body.collision_shape)
                    fo.write("\t\t\"mass\":%f,\n" % obj.rigid_body.mass)
                    fo.write("\t\t\"friction\":%f,\n" % obj.rigid_body.friction)
                    fo.write("\t\t\"restitution\":%f,\n" % obj.rigid_body.restitution)
                    fo.write("\t\t\"xpos\":%f,\n" % obj.location[1])
                    fo.write("\t\t\"ypos\":%f,\n" % obj.location[2])
                    fo.write("\t\t\"zpos\":%f,\n" % obj.location[0])
                    fo.write("\t\t\"wqat\":%f,\n" % obj.rotation_quaternion[0])
                    fo.write("\t\t\"xqat\":%f,\n" % obj.rotation_quaternion[2])
                    fo.write("\t\t\"yqat\":%f,\n" % obj.rotation_quaternion[3])
                    fo.write("\t\t\"zqat\":%f,\n" % obj.rotation_quaternion[1])
                    fo.write("\t\t\"xscl\":%f,\n" % obj.scale[1])
                    fo.write("\t\t\"yscl\":%f,\n" % obj.scale[2])
                    fo.write("\t\t\"zscl\":%f,\n" % obj.scale[0])

                    if obj.rigid_body.collision_shape == "CONVEX_HULL":
                        fo.write("\t\t\"vertex\": [\n")

                        data = obj.data
                        polys = data.polygons
                        verts = data.vertices

                        colorList = []
                        cordList  = []
                        normList  = []

                        for material in data.materials:
                            colorList.append(material.diffuse_color)

                        for vert in verts:
                            cordList.append(vert.co)
                            normList.append(vert.normal)



                        isFirst = True
                        for poly in polys:
                            if isFirst == True:
                                isFirst = False;
                            else:
                                fo.write(",\n\n")

                            fo.write("\t\t\t[%f, %f, %f, %f, %f, %f, %f, %f, %f],\n" % (
                                                                            cordList[poly.vertices[0]][1],
                                                                            cordList[poly.vertices[0]][2],
                                                                            cordList[poly.vertices[0]][0],
                                                                            normList[poly.vertices[0]][1],
                                                                            normList[poly.vertices[0]][2],
                                                                            normList[poly.vertices[0]][0],
                                                                            colorList[poly.material_index][0],
                                                                            colorList[poly.material_index][1],
                                                                            colorList[poly.material_index][2]))

                            fo.write("\t\t\t[%f, %f, %f, %f, %f, %f, %f, %f, %f],\n" % (
                                                                            cordList[poly.vertices[1]][1],
                                                                            cordList[poly.vertices[1]][2],
                                                                            cordList[poly.vertices[1]][0],
                                                                            normList[poly.vertices[1]][1],
                                                                            normList[poly.vertices[1]][2],
                                                                            normList[poly.vertices[1]][0],
                                                                            colorList[poly.material_index][0],
                                                                            colorList[poly.material_index][1],
                                                                            colorList[poly.material_index][2]))

                            fo.write("\t\t\t[%f, %f, %f, %f, %f, %f, %f, %f, %f]" % (
                                                                            cordList[poly.vertices[2]][1],
                                                                            cordList[poly.vertices[2]][2],
                                                                            cordList[poly.vertices[2]][0],
                                                                            normList[poly.vertices[2]][1],
                                                                            normList[poly.vertices[2]][2],
                                                                            normList[poly.vertices[2]][0],
                                                                            colorList[poly.material_index][0],
                                                                            colorList[poly.material_index][1],
                                                                            colorList[poly.material_index][2]))

                        fo.write("\n")

                        fo.write("\t\t]\n")


                    fo.write("\t},\n")

                else:

                    fo.write("\t{\n")
                    fo.write("\t\t\"objectType\":\"MESH\",\n")
                    fo.write("\t\t\"name\":\"%s\",\n" % keyname)
                    fo.write("\t\t\"xpos\":%f,\n" % obj.location[1])
                    fo.write("\t\t\"ypos\":%f,\n" % obj.location[2])
                    fo.write("\t\t\"zpos\":%f,\n" % obj.location[0])
                    fo.write("\t\t\"wqat\":%f,\n" % obj.rotation_quaternion[0])
                    fo.write("\t\t\"xqat\":%f,\n" % obj.rotation_quaternion[2])
                    fo.write("\t\t\"yqat\":%f,\n" % obj.rotation_quaternion[3])
                    fo.write("\t\t\"zqat\":%f,\n" % obj.rotation_quaternion[1])
                    fo.write("\t\t\"xscl\":%f,\n" % obj.scale[1])
                    fo.write("\t\t\"yscl\":%f,\n" % obj.scale[2])
                    fo.write("\t\t\"zscl\":%f,\n" % obj.scale[0])


                    fo.write("\t\t\"vertex\": [\n")

                    data = obj.data
                    polys = data.polygons
                    verts = data.vertices

                    colorList = []
                    cordList  = []
                    normList  = []

                    for material in data.materials:
                        colorList.append(material.diffuse_color)

                    for vert in verts:
                        cordList.append(vert.co)
                        normList.append(vert.normal)



                    isFirst = True
                    for poly in polys:
                        if isFirst == True:
                            isFirst = False;
                        else:
                            fo.write(",\n\n")

                        fo.write("\t\t\t[%f, %f, %f, %f, %f, %f, %f, %f, %f],\n" % (
                                                                        cordList[poly.vertices[0]][1],
                                                                        cordList[poly.vertices[0]][2],
                                                                        cordList[poly.vertices[0]][0],
                                                                        normList[poly.vertices[0]][1],
                                                                        normList[poly.vertices[0]][2],
                                                                        normList[poly.vertices[0]][0],
                                                                        colorList[poly.material_index][0],
                                                                        colorList[poly.material_index][1],
                                                                        colorList[poly.material_index][2]))

                        fo.write("\t\t\t[%f, %f, %f, %f, %f, %f, %f, %f, %f],\n" % (
                                                                        cordList[poly.vertices[1]][1],
                                                                        cordList[poly.vertices[1]][2],
                                                                        cordList[poly.vertices[1]][0],
                                                                        normList[poly.vertices[1]][1],
                                                                        normList[poly.vertices[1]][2],
                                                                        normList[poly.vertices[1]][0],
                                                                        colorList[poly.material_index][0],
                                                                        colorList[poly.material_index][1],
                                                                        colorList[poly.material_index][2]))

                        fo.write("\t\t\t[%f, %f, %f, %f, %f, %f, %f, %f, %f]" % (
                                                                        cordList[poly.vertices[2]][1],
                                                                        cordList[poly.vertices[2]][2],
                                                                        cordList[poly.vertices[2]][0],
                                                                        normList[poly.vertices[2]][1],
                                                                        normList[poly.vertices[2]][2],
                                                                        normList[poly.vertices[2]][0],
                                                                        colorList[poly.material_index][0],
                                                                        colorList[poly.material_index][1],
                                                                        colorList[poly.material_index][2]))

                    fo.write("\n")

                    fo.write("\t\t]\n")

                    fo.write("\t},\n")



        fo.write("\t{\n")
        fo.write("\t\t\"objectType\":\"DUMMY\"\n")
        fo.write("\t}\n")

        fo.write("]\n")


        fo.close()


        return {'FINISHED'}


def menu_func(self, context):
    self.layout.operator(fpmExporter.bl_idname, text="Friends Physical Model (.fpm)")

def unregister():
    bpy.utils.unregister_module(__name__)
    bpy.types.INFO_MT_file_export.remove(menu_func)

def register():
    bpy.utils.register_module(__name__)
    bpy.types.INFO_MT_file_export.append(menu_func)
