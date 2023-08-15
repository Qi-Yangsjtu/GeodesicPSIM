import sys
import pandas as pd
import numpy as np
import os
import argparse

def generate_dir(path):
    if not os.path.exists(path):
        os.mkdir(path)
    return path

def cal_obj(ref_obj_path, ref_img_path, dis_obj_path, dis_img_path,  results_path, results_file_name, N):
    ref_obj_file = []
    ref_img_file = []
    dis_obj_file = []
    dis_img_file = []
    for filename in os.listdir(ref_obj_path):
        if filename.endswith('.obj'):
            ref_obj_file.append(os.path.join(ref_obj_path,filename))
    for filename in os.listdir(ref_img_path):
        if filename.endswith('.png') or filename.endswith('.jpg'):
            ref_img_file.append(os.path.join(ref_img_path,filename))
    for filename in os.listdir(dis_obj_path):
        if filename.endswith('.obj'):
            dis_obj_file.append(os.path.join(dis_obj_path,filename))
    for filename in os.listdir(dis_img_path):
        if filename.endswith('.png') or filename.endswith('.jpg'):
            dis_img_file.append(os.path.join(dis_img_path,filename))

    ref_obj_file.sort()
    ref_img_file.sort()
    dis_obj_file.sort()
    dis_img_file.sort()
    print("list of reference mesh file:")
    print(ref_obj_file)
    print("list of reference mesh texture map file:")
    print(ref_img_file)
    print("list of distorted mesh file:")
    print(dis_obj_file)
    print("list of distorted mesh texture map file:")
    print(dis_img_file)

    if len(ref_obj_file)!=len(ref_img_file):
        print("The number of reference mesh not equal to the number of texture map, please check! ")
        sys.exit(0)
    else:
        print("The frame of reference mesh: %d"%(len(ref_obj_file)))

    if len(dis_obj_file)!=len(dis_img_file):
        print("The number of distorted mesh not equal to the number of texture map, please check! ")
        sys.exit(0)
    else:
        print("The frame of distorted mesh: %d"%(len(dis_obj_file)))
    if len(ref_obj_file)!=len(dis_img_file):
        print("The number of reference mesh not equal to the number of distorted mesh, please check! ")
        sys.exit(0)

    exe = 'GeodesicPSIM.exe'

    results_file = os.path.join(results_path, results_file_name)
    if (os.path.exists(results_file)):
        print("The result file (%s) already exists, please use a new file name to store results!"%(results_file))
        sys.exit(0)



    for i in range(0,len(ref_obj_file)):
        print("%dth reference sample: %s" % (i+1, ref_obj_file[i]))
        print("%dth texture map of reference sample: %s" % (i+1, ref_img_file[i]))
        print("%dth distorted sample: %s" % (i+1, dis_obj_file[i]))
        print("%dth texture mpa of distorted sample: %s" % (i+1, dis_img_file[i]))

        cmd = '{} {} {} {} {} --N {} --result_file {}'.format(exe, ref_obj_file[i], ref_img_file[i],
                                                                 dis_obj_file[i], dis_img_file[i], N,
                                                                 results_file)

        print(cmd)
        os.system(cmd)







def main(config):
    ref_obj_path = config.ref_obj_path
    ref_img_path = config.ref_img_path
    dis_obj_path = config.dis_obj_path
    dis_img_path = config.dis_img_path
    results_path = config.results_path
    generate_dir(results_path)
    results_file_name = config.results_file_name
    N = config.N
    cal_obj(ref_obj_path, ref_img_path, dis_obj_path, dis_img_path,  results_path, results_file_name, N)



if __name__ == '__main__':

    parser = argparse.ArgumentParser()


    parser.add_argument('--ref_obj_path', type=str, default = './testSample/reference/')
    parser.add_argument('--ref_img_path', type=str, default = './testSample/reference/')
    parser.add_argument('--dis_obj_path', type=str, default = './testSample/distorted/')
    parser.add_argument('--dis_img_path', type=str, default = './testSample/distorted/')
    parser.add_argument('--N', type=int, default = 500)
    parser.add_argument('--results_path', type=str, default = './testSample/score/')
    parser.add_argument('--results_file_name', type=str, default='result_GeodesicPSIM.csv')
    config = parser.parse_args()

    main(config)
