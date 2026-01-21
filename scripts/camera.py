import argparse,subprocess

import os,json,yaml

def auto_update_config(args):
    camera_config_path = args.file
    with open(camera_config_path, 'r') as f:
        camera_json = json.load(f)

    for i,config in enumerate(camera_json['cameras']):
        if(config['type'] == 'mono'):
            continue
        topic = config['topic']['camera_info']
        cmd = f"source /opt/ros/humble/setup.bash && ros2 topic echo {topic} --once"
        try:
            result = subprocess.run(
                cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=10
            )
            result = result.stdout.strip()[:-4]
            for index,line in enumerate(result.split('\n')):
                if line.startswith("header"):
                    result = "\n".join(result.split('\n')[index:])
                    break
            result = yaml.safe_load(result)
            shape = [result["width"], result["height"]]
            calib = {
                "fx": result["k"][0],
                "fy": result["k"][4],
                "cx": result["k"][2],
                "cy": result["k"][5],
                "baseline": 0.0699967
            }
            arr_d = result["d"]
            arr_k = result["k"]
            arr_r = result["r"]
            arr_p = result["p"]

            camera_json['cameras'][i]['shape'] = shape
            camera_json['cameras'][i]['calib'] = calib
            camera_json['cameras'][i]['distortion'] = {
                "arr_d": arr_d,
                "arr_k": arr_k,
                "arr_r": arr_r,
                "arr_p": arr_p
            }
        except Exception as e:
            print(f" please make sure the topic {topic} is exist")
        finally:
            continue
        
    with open(camera_config_path, 'w') as f:
        json.dump(camera_json, f, indent=4, sort_keys=False)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, default='', help='store config path')
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    auto_update_config(args)