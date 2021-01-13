
import base64
from pygltflib import *  # DATA_URI_HEADER, Buffer, GLTF2
import struct
import os
import glob
# from pyquaternion import Quaternion
import argparse
from argparse import ArgumentParser, Namespace

def inspect_accessor():

    filename = "/home/eundersander/projects/zatun/cleaned_blender/floor_test_processed.gltf"

    gltf = GLTF2().load(filename)
    mesh = gltf.meshes[gltf.scenes[gltf.scene].nodes[2]]

    # get the vertices for each primitive in the mesh (in this example there is only one)
    for primitive in mesh.primitives:

        # get the binary data for this mesh primitive from the buffer
        accessor = gltf.accessors[primitive.attributes.TANGENT]
        size_of_prim = 16
        unpack_string = "<ffff"
        bufferView = gltf.bufferViews[accessor.bufferView]
        buffer = gltf.buffers[bufferView.buffer]
        data = gltf.decode_data_uri(buffer.uri)
        print("len(data) on inspect: {}".format(len(data)))
        num_verts = accessor.count

        for i in range(num_verts):
            index = bufferView.byteOffset + accessor.byteOffset + i*size_of_prim  # the location in the buffer of this vertex
            d = data[index:index+size_of_prim]  # the vertex data
            v = struct.unpack(unpack_string, d)   # convert from base64 to k floats
            print(i, v)

def modify_tangents():


    filename = "/home/eundersander/projects/zatun/cleaned_blender/floor_test_with_tangents_without_diffuse_embedded.gltf"

    gltf = GLTF2().load(filename)
    mesh = gltf.meshes[gltf.scenes[gltf.scene].nodes[2]]

    # get the vertices for each primitive in the mesh (in this example there is only one)
    for primitive in mesh.primitives:

        # get the binary data for this mesh primitive from the buffer
        accessor = gltf.accessors[primitive.attributes.TANGENT]
        size_of_prim = 16
        unpack_string = "<ffff"
        bufferView = gltf.bufferViews[accessor.bufferView]
        buffer = gltf.buffers[bufferView.buffer]
        data = gltf.decode_data_uri(buffer.uri)
        data_bytearray = bytearray(data)
        num_verts = accessor.count

        for i in range(num_verts):
            index = bufferView.byteOffset + accessor.byteOffset + i*size_of_prim  # the location in the buffer of this vertex
            d = data_bytearray[index:index+size_of_prim]  # the vertex data
            v = struct.unpack(unpack_string, d)   # convert from base64 to k floats

            # flip tangent vector
            temp = struct.pack(unpack_string, v[0], v[1], v[2], v[3])
            # temp = struct.pack(unpack_string, 1, 2, 3, 4)
            data_bytearray[index:index+size_of_prim] = temp

        new_encoded_data = base64.b64encode(data_bytearray).decode('utf-8')
        buffer.uri = f'{DATA_URI_HEADER}{new_encoded_data}'

    gltf.save("/home/eundersander/projects/zatun/cleaned_blender/floor_test_processed.gltf") 

    # gltf.convert_buffers(BufferFormat.BINFILE)
    # gltf.save("/home/eundersander/projects/zatun/cleaned_blender/floor_test_processed_bin.gltf") 

def add_texcoord1():

    filename = "/home/eundersander/projects/zatun/cleaned_blender/floor_test_embedded.gltf"

    gltf = GLTF2().load(filename)
    mesh = gltf.meshes[gltf.scenes[gltf.scene].nodes[0]]

    # get the vertices for each primitive in the mesh (in this example there is only one)
    for primitive in mesh.primitives:

        # get the binary data for this mesh primitive from the buffer
        accessor = gltf.accessors[primitive.attributes.TEXCOORD_0]
        size_of_prim = 8
        unpack_string = "<ff"
        bufferView = gltf.bufferViews[accessor.bufferView]
        buffer = gltf.buffers[bufferView.buffer]
        print("len(buffer.uri): {}".format(len(buffer.uri)))
        data = gltf.decode_data_uri(buffer.uri)
        print("len(data) before insertion: {}".format(len(data)))
        offset_to_new_data = len(data)
        num_verts = accessor.count

        for i in range(num_verts):
            index = bufferView.byteOffset + accessor.byteOffset + i*size_of_prim  # the location in the buffer of this vertex
            d = data[index:index+size_of_prim]  # the vertex data
            v = struct.unpack(unpack_string, d)   # convert from base64 to k floats

            # temp exercise: swap U and V here
            temp = struct.pack(unpack_string, v[1], v[0])
            assert(len(temp) == size_of_prim)
            # append to existing bytearray
            data += temp
            # new_data_bytearray[index:index+size_of_prim] = temp

            # struct.pack_into(unpack_string, new_data_bytearray, index, v[1], v[0])

            # vertices.append(v)
            # print(i, v)

        print("len(data) after insertion: {}".format(len(data)))

        new_encoded_data = base64.b64encode(data).decode('utf-8')
        new_buffer = Buffer()
        new_buffer.uri = f'{DATA_URI_HEADER}{new_encoded_data}'
        print("len(new_buffer.uri): {}".format(len(new_buffer.uri)))

        gltf.buffers[bufferView.buffer] = new_buffer

        bufferView2 = BufferView()
        bufferView2.buffer = bufferView.buffer
        bufferView2.byteOffset = offset_to_new_data
        bufferView2.byteLength = num_verts * size_of_prim
        assert len(data) == bufferView2.byteOffset + bufferView2.byteLength
        assert bufferView2.byteLength == bufferView.byteLength
        bufferView2.target = bufferView.target
        
        bufferView2_index = len(gltf.bufferViews)
        gltf.bufferViews.append(bufferView2)

        accessor2 = Accessor()
        accessor2.bufferView = bufferView2_index
        accessor2.byteOffset = 0
        accessor2.componentType = FLOAT
        accessor2.count = accessor.count
        accessor2.type = accessor.type
        accessor2.max = [1.0, 1.0]
        accessor2.min = [0.0, 0.0]

        primitive.attributes.TEXCOORD_1 = len(gltf.accessors)
        gltf.accessors.append(accessor2)

    gltf.save("/home/eundersander/projects/zatun/cleaned_blender/floor_test_processed.gltf") 

    # gltf.convert_buffers(BufferFormat.BINFILE)

    # gltf.save("/home/eundersander/projects/zatun/cleaned_blender/floor_test_processed_bin.gltf") 

def get_filepaths(directory, pattern):
    filepaths = []
    os.chdir(directory)
    for filepath in glob.glob(pattern):
        filepaths.append(filepath)
    return filepaths


def change_pbr_materials():

    filepaths = get_filepaths("/home/eundersander/projects/zatun/Zatun_Final/Zatun_Final/glb/frl_apartment/", "*.glb")

    output_folder = "/home/eundersander/projects/zatun/change_pbr_material_output/"

    for filepath in filepaths:
        gltf = GLTF2().load(filepath)
        for material in gltf.materials:
            material.pbrMetallicRoughness.metallicFactor = 0
            material.pbrMetallicRoughness.roughnessFactor = 1

        base_name = os.path.basename(filepath)
        gltf.save(output_folder + base_name) 
        print("{}: modified {} materials".format(base_name, len(gltf.materials)))


def convert_gltf_type():
    
    filename = "/home/eundersander/projects/zatun/Zatun_Final/Zatun_Final/glb/frl_apartment/frl_apartment_0.glb"

    gltf = GLTF2().load(filename)

    gltf.convert_buffers(BufferFormat.BINFILE)

    gltf.save("/home/eundersander/projects/zatun/cleaned_blender/frl_apartment_0_rough_nonmetal.glb") 


def to_quat(rotation):

    return Quaternion(rotation[3], rotation[0], rotation[1], rotation[2])
    # return Quaternion(*rotation)

def to_gltf_rotation(quat):

    return [quat.elements[1], quat.elements[2], quat.elements[3], quat.elements[0]]
    # return list(quat.elements)


def fix_rotation():

    filepaths = get_filepaths("/home/eundersander/projects/zatun/1_change_pbr_material_output/", "*.glb")

    output_folder = "/home/eundersander/projects/zatun/2_remove_wall_transform/"

    for filepath in filepaths:

        gltf = GLTF2().load(filepath)

        q_fixup = None
        q_stage = None
        t_fixup = None
        t_stage = None

        for node in gltf.nodes:

            if node.name == "frl_apartment_wall":

                t_stage = list(node.translation)
                if not t_stage:
                    t_stage = [0, 0, 0]
                t_fixup = [-x for x in t_stage]

                q_stage = to_quat(node.rotation)
                q_fixup = q_stage.inverse

                q = Quaternion(*node.rotation)
                q_fixed = q_fixup * q
                temp_rotation = to_gltf_rotation(q_fixed)

        assert q_fixup is not None

        for node in gltf.nodes:

            q = to_quat(node.rotation)
            t = node.translation
            if not t:
                t = [0, 0, 0]
            q_fixed = q_fixup * q
            t_rel = [a - b for (a,b) in zip(t, t_stage)]
            t_rel_prime = q_fixup.rotate(t_rel)

            node.translation = t_rel_prime
            node.rotation = to_gltf_rotation(q_fixed)

        base_name = os.path.basename(filepath)
        print("fixed rotation for {}".format(base_name))
        gltf.save(output_folder + base_name) 



def fix_rotation_for_lights():

    # filepath = "/home/eundersander/projects/zatun/1_change_pbr_material_output/frl_apartment_0.glb"

    # gltf = GLTF2().load(filepath)

    # q_fixup = None
    # q_stage = None
    # t_fixup = None
    # t_stage = None

    # for node in gltf.nodes:

    #     if node.name == "frl_apartment_wall":

    #         t_stage = list(node.translation)
    #         if not t_stage:
    #             t_stage = [0, 0, 0]
    #         t_fixup = [-x for x in t_stage]

    #         q_stage = to_quat(node.rotation)
    #         q_fixup = q_stage.inverse

    #         q = Quaternion(*node.rotation)
    #         q_fixed = q_fixup * q
    #         temp_rotation = to_gltf_rotation(q_fixed)

    t_stage = [1.2,-1.6,-1.6]
    q_fixup = Quaternion(axis=[0,1,0], degrees=270)

    assert q_fixup is not None

    import json

    print("q_fixup: {}".format(q_fixup))

    json_input_filepath = "/home/eundersander/projects/habitat-sim2/frl_apartment_1.json"
    json_output_filepath = "/home/eundersander/projects/habitat-sim2/my_lights.json"

    with open(json_input_filepath) as json_file:
        data = json.load(json_file)
        for l in data['lights']:
            print(l)
            t = l["position"]
            t_rel = [a - b for (a,b) in zip(t, t_stage)]
            t_rel_prime = q_fixup.rotate(t_rel)
            pos = list(t_rel_prime)
            l["position"] = pos
            print(l)
            print("")

    with open(json_output_filepath, 'w') as outfile:
        json.dump(data, outfile)


def get_data(gltf, buffer):

    current_buffer_format = gltf.identify_uri(buffer.uri)
    if current_buffer_format == BufferFormat.BINFILE:
        data = gltf.load_file_uri(buffer.uri)
    elif current_buffer_format == BufferFormat.DATAURI:
        data = gltf.decode_data_uri(buffer.uri)
    elif current_buffer_format == BufferFormat.BINARYBLOB:
        data = gltf.binary_blob()
    return data


def check_for_errors(path):

    # get gltf and glb files
    filepaths = get_filepaths(path, "*.gl*")

    if not filepaths:
        print("no GLTF or GLB files found in directory [{}]".format(path))
        return

    for filepath in filepaths:
        gltf = GLTF2().load(filepath)
        base_name = os.path.basename(filepath)
        filedir = os.path.dirname(filepath)
        print(base_name)

        found_error = False

        # check for existence of tangents
        for mesh in gltf.meshes:
            for primitive in mesh.primitives:
                if not primitive.attributes.TANGENT:
                    print("  mesh {}: no tangents".format(mesh.name))
                    found_error = True        
                else:
                    accessor = gltf.accessors[primitive.attributes.TANGENT]
                    num_verts = accessor.count
                    if num_verts == 0:
                        print("  mesh {}: zero tangents".format(mesh.name))
                        found_error = True        

        # check scale
        for mesh in gltf.meshes:
            for primitive in mesh.primitives:
                accessor = gltf.accessors[primitive.attributes.POSITION]
                size_of_prim = 12
                unpack_string = "<fff"
                bufferView = gltf.bufferViews[accessor.bufferView]
                buffer = gltf.buffers[bufferView.buffer]
                data = get_data(gltf, buffer)  # gltf.decode_data_uri(buffer.uri) if buffer.uri else gltf.binary_blob()  # not sure about this
                num_verts = accessor.count

                if num_verts:
                    extents_min = None
                    extents_max = None
                    for i in range(num_verts):
                        index = bufferView.byteOffset + accessor.byteOffset + i*size_of_prim  # the location in the buffer of this vertex
                        d = data[index:index+size_of_prim]  # the vertex data
                        v = struct.unpack(unpack_string, d)   # convert from base64 to k floats
                        if not extents_min:
                            extents_min = list(v)
                            extents_max = list(v)
                        else:
                            for i in range(len(extents_min)):
                                extents_min[i] = min(v[i], extents_min[i])
                                extents_max[i] = max(v[i], extents_max[i])

                    max_dim = 0
                    max_dist_from_origin = 0
                    for i in range(len(extents_min)):
                        max_dim = max(max_dim, extents_max[i] - extents_min[i])
                        max_dist_from_origin = max(max_dist_from_origin, max(abs(extents_min[i]), abs(extents_max[i])))

                    if max_dim > 20:
                        print("  mesh {}: large dimension of {:.1f} meters; possible wrong scale".format(mesh.name, max_dim))
                    if max_dist_from_origin > 200:
                        print("  mesh {}: a vertex is {:.1f} meters from the origin; possible wrong scale".format(mesh.name, max_dist_from_origin))

        # check PBR material
        for material in gltf.materials:
            if not material.pbrMetallicRoughness:
                print("  material {}: no pbr material".format(material.name))
                found_error = True
            else:
                if not material.pbrMetallicRoughness.baseColorTexture:
                    print("  material {}: empty PBR baseColorTexture".format(material.name))
                    found_error = True
                if not material.pbrMetallicRoughness.metallicRoughnessTexture:
                    print("  material {}: empty PBR metallicRoughnessTexture".format(material.name))
                    found_error = True
                else:
                    tex = gltf.textures[material.pbrMetallicRoughness.metallicRoughnessTexture.index]
                    # todo: use source to look up into images?
                    image = gltf.images[tex.source]
                    if image.uri:
                        image_filepath = os.path.join(filedir, image.uri)
                        if not os.path.exists(image_filepath):
                            print("  material {}: missing file {}".format(material.name, image_filepath))
                            found_error = True
                    elif not image.bufferView:
                        print("  material {}: no metallicRoughnessTexture image uri or bufferView".format(material.name))
                        found_error = True
                    pass
            if not material.normalTexture:
                print("  material {}: empty normalTexture".format(material.name))
                found_error = True

        if not found_error:
            print("  ok!")

def create_arg_parser() -> ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument('path',
                        metavar='path',
                        type=str,
                        help='a folder to process')
    return parser

def main():
    args = create_arg_parser().parse_args()
    check_for_errors(args.path)

if __name__ == "__main__":
    main()

# add_texcoord1()
# inspect_accessor()
# modify_tangents()
# inspect_accessor()
# convert_gltf_type()
# fix_rotation()
# fix_rotation_for_lights()
# check_for_errors()