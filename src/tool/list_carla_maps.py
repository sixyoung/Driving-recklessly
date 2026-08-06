#!/usr/bin/env python3

import carla
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description='CARLA Map Manager')
    parser.add_argument('--load', type=str, help='Load specific map (e.g., /Game/Package01/Maps/Map02/Map02)')
    args = parser.parse_args()

    try:
        # 连接到CARLA服务器
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        
        # 获取可用地图列表
        available_maps = client.get_available_maps()
        
        print("\n=== CARLA Available Maps ===")
        print(f"Total maps: {len(available_maps)}")
        print("\nMap List:")
        for i, map_name in enumerate(available_maps, 1):
            print(f"{i}. {map_name}")

        # 如果指定了要加载的地图
        if args.load:
            if args.load in available_maps:
                print(f"\nLoading map: {args.load}")
                client.load_world(args.load)
                print("Map loaded successfully!")
            else:
                print(f"\nError: Map '{args.load}' not found in available maps!")
                sys.exit(1)
            
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main() 