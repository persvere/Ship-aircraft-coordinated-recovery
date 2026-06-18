"""
ArUco标记生成器
用法: python generate_aruco_markers.py
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import argparse
from datetime import datetime
import json

class ArUcoGenerator:
    def __init__(self, output_dir="D:/unity/wave_t/Assets/Models/aruco target"):
        self.output_dir = output_dir
        self.ensure_directory()
        
        # 可用的字典类型
        self.dictionaries = {
            '4x4_50': aruco.DICT_4X4_50,
            '5x5_50': aruco.DICT_5X5_50,
            '6x6_250': aruco.DICT_6X6_250,
            '7x7_1000': aruco.DICT_7X7_1000
        }
    
    def ensure_directory(self):
        """确保输出目录存在"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"创建目录: {self.output_dir}")
    
    def generate_single_marker(self, marker_id, dict_type='6x6_250', 
                               size_px=512, border_bits=1, 
                               show_preview=False, save=True):
        """
        生成单个ArUco标记
        
        参数:
            marker_id: 标记ID
            dict_type: 字典类型 ('4x4_50', '5x5_50', '6x6_250', '7x7_1000')
            size_px: 图片尺寸（像素）
            border_bits: 边框大小
            show_preview: 是否显示预览
            save: 是否保存为文件
        """
        # 获取字典
        if dict_type not in self.dictionaries:
            print(f"错误: 不支持的字典类型: {dict_type}")
            return None
        
        aruco_dict = aruco.getPredefinedDictionary(self.dictionaries[dict_type])
        
        # 生成标记
        marker_image = np.zeros((size_px, size_px), dtype=np.uint8)
        marker_image = aruco.generateImageMarker(aruco_dict, marker_id, 
                                                size_px, marker_image, border_bits)
        
        # 显示预览
        if show_preview:
            cv2.imshow(f'ArUco Marker ID:{marker_id}', marker_image)
            cv2.waitKey(500)  # 显示0.5秒
            cv2.destroyAllWindows()
        
        # 保存文件
        if save:
            filename = f"aruco_{dict_type}_id{marker_id:03d}.png"
            filepath = os.path.join(self.output_dir, filename)
            cv2.imwrite(filepath, marker_image)
            print(f"保存: {filename}")
            
            # 同时保存一份JSON元数据
            self.save_metadata(marker_id, dict_type, size_px, border_bits, filepath)
        
        return marker_image
    
    def generate_multiple_markers(self, marker_ids, dict_type='6x6_250', 
                                 size_px=512, border_bits=1):
        """生成多个标记"""
        for marker_id in marker_ids:
            self.generate_single_marker(marker_id, dict_type, size_px, border_bits)
    
    def generate_random_markers(self, count=10, dict_type='6x6_250',
                               size_px=512, border_bits=1):
        """生成随机标记"""
        # 根据字典类型确定最大ID
        max_id = self.get_max_id_for_dict(dict_type)
        
        # 生成不重复的随机ID
        np.random.seed(int(datetime.now().timestamp()))  # 使用时间作为随机种子
        random_ids = np.random.choice(max_id, min(count, max_id), replace=False)
        
        print(f"生成 {len(random_ids)} 个随机标记:")
        for marker_id in random_ids:
            self.generate_single_marker(int(marker_id), dict_type, size_px, border_bits)
    
    def generate_range_markers(self, start_id, end_id, dict_type='6x6_250',
                              size_px=512, border_bits=1):
        """生成指定范围的标记"""
        max_id = self.get_max_id_for_dict(dict_type)
        end_id = min(end_id, max_id-1)
        
        for marker_id in range(start_id, end_id + 1):
            self.generate_single_marker(marker_id, dict_type, size_px, border_bits)
    
    def get_max_id_for_dict(self, dict_type):
        """获取字典支持的最大ID"""
        dict_info = {
            '4x4_50': 50,
            '5x5_50': 50,
            '6x6_250': 250,
            '7x7_1000': 1000
        }
        return dict_info.get(dict_type, 250)
    
    def save_metadata(self, marker_id, dict_type, size_px, border_bits, filepath):
        """保存标记的元数据"""
        metadata = {
            'id': marker_id,
            'dictionary': dict_type,
            'size_px': size_px,
            'border_bits': border_bits,
            'date_created': datetime.now().isoformat(),
            'filepath': filepath
        }
        
        # 保存到JSON文件
        json_path = filepath.replace('.png', '.json')
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def list_generated_markers(self):
        """列出已生成的标记"""
        png_files = [f for f in os.listdir(self.output_dir) if f.endswith('.png')]
        
        if not png_files:
            print("没有找到标记文件")
            return
        
        print(f"找到 {len(png_files)} 个标记文件:")
        for file in sorted(png_files):
            print(f"  - {file}")
    
    def cleanup_directory(self, confirm=True):
        """清理目录中的所有PNG文件"""
        png_files = [f for f in os.listdir(self.output_dir) if f.endswith('.png')]
        
        if not png_files:
            print("没有文件可清理")
            return
        
        if confirm:
            response = input(f"确定要删除 {len(png_files)} 个PNG文件吗? (y/n): ")
            if response.lower() != 'y':
                print("取消清理")
                return
        
        for file in png_files:
            filepath = os.path.join(self.output_dir, file)
            os.remove(filepath)
            print(f"删除: {file}")
            
            # 同时删除对应的JSON文件
            json_path = filepath.replace('.png', '.json')
            if os.path.exists(json_path):
                os.remove(json_path)
        
        print(f"已清理 {len(png_files)} 个文件")
    
    def create_marker_sheet(self, marker_ids, dict_type='6x6_250', 
                           marker_size=200, cols=5, spacing=20):
        """
        创建标记表格（多个标记在一张图上）
        """
        rows = (len(marker_ids) + cols - 1) // cols
        sheet_width = cols * (marker_size + spacing) - spacing
        sheet_height = rows * (marker_size + spacing) - spacing
        
        sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255
        
        for i, marker_id in enumerate(marker_ids):
            row = i // cols
            col = i % cols
            
            x = col * (marker_size + spacing)
            y = row * (marker_size + spacing)
            
            # 生成单个标记
            marker = self.generate_single_marker(marker_id, dict_type, 
                                                marker_size, 1, False, False)
            
            # 粘贴到表格
            sheet[y:y+marker_size, x:x+marker_size] = marker
            
            # 添加ID标签
            self._add_id_label(sheet, x, y, marker_size, marker_id)
        
        # 保存表格
        filename = f"aruco_sheet_{dict_type}.png"
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, sheet)
        print(f"保存标记表格: {filename}")
        
        return sheet
    
    def _add_id_label(self, image, x, y, size, marker_id):
        """在标记下方添加ID标签"""
        import cv2
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        
        text = f"ID:{marker_id}"
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        
        text_x = x + (size - text_size[0]) // 2
        text_y = y + size + 15
        
        cv2.putText(image, text, (text_x, text_y), font, font_scale, 0, thickness)

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='ArUco标记生成器')
    parser.add_argument('--output', '-o', default='D:/unity/wave_t/Assets/Models/aruco target',
                       help='输出目录')
    parser.add_argument('--dict', '-d', default='6x6_250',
                       choices=['4x4_50', '5x5_50', '6x6_250', '7x7_1000'],
                       help='字典类型')
    parser.add_argument('--size', '-s', type=int, default=512,
                       help='标记尺寸（像素）')
    parser.add_argument('--border', '-b', type=int, default=1,
                       help='边框大小')
    
    subparsers = parser.add_subparsers(dest='command', help='子命令')
    
    # 单标记生成
    parser_single = subparsers.add_parser('single', help='生成单个标记')
    parser_single.add_argument('id', type=int, help='标记ID')
    
    # 批量生成
    parser_batch = subparsers.add_parser('batch', help='生成批量标记')
    parser_batch.add_argument('start', type=int, help='起始ID')
    parser_batch.add_argument('end', type=int, help='结束ID')
    
    # 随机生成
    parser_random = subparsers.add_parser('random', help='生成随机标记')
    parser_random.add_argument('count', type=int, default=10, nargs='?',
                              help='标记数量')
    
    # 列表
    subparsers.add_parser('list', help='列出已生成标记')
    
    # 清理
    subparsers.add_parser('clean', help='清理目录')
    
    # 表格
    parser_sheet = subparsers.add_parser('sheet', help='生成标记表格')
    parser_sheet.add_argument('ids', type=int, nargs='+', help='标记ID列表')
    parser_sheet.add_argument('--cols', type=int, default=5, help='列数')
    
    args = parser.parse_args()
    
    # 创建生成器
    generator = ArUcoGenerator(args.output)
    
    if args.command == 'single':
        generator.generate_single_marker(
            args.id, args.dict, args.size, args.border, 
            show_preview=True, save=True
        )
    
    elif args.command == 'batch':
        generator.generate_range_markers(
            args.start, args.end, args.dict, 
            args.size, args.border
        )
    
    elif args.command == 'random':
        generator.generate_random_markers(
            args.count, args.dict, args.size, args.border
        )
    
    elif args.command == 'list':
        generator.list_generated_markers()
    
    elif args.command == 'clean':
        generator.cleanup_directory()
    
    elif args.command == 'sheet':
        generator.create_marker_sheet(
            args.ids, args.dict, marker_size=200, cols=args.cols
        )
    
    else:
        # 默认交互模式
        interactive_mode(generator)

def interactive_mode(generator):
    """交互模式"""
    print("=" * 50)
    print("ArUco标记生成器 - 交互模式")
    print("=" * 50)
    
    while True:
        print("\n请选择操作:")
        print("1. 生成单个标记")
        print("2. 生成范围标记")
        print("3. 生成随机标记")
        print("4. 生成标记表格")
        print("5. 列出已生成标记")
        print("6. 清理目录")
        print("7. 退出")
        
        choice = input("\n请输入选项 (1-7): ").strip()
        
        if choice == '1':
            marker_id = int(input("请输入标记ID: "))
            dict_type = input("字典类型 (默认6x6_250): ") or '6x6_250'
            size = int(input("标记尺寸 (默认512): ") or 512)
            generator.generate_single_marker(marker_id, dict_type, size, show_preview=True)
        
        elif choice == '2':
            start = int(input("起始ID: "))
            end = int(input("结束ID: "))
            dict_type = input("字典类型 (默认6x6_250): ") or '6x6_250'
            generator.generate_range_markers(start, end, dict_type)
        
        elif choice == '3':
            count = int(input("生成数量 (默认10): ") or 10)
            dict_type = input("字典类型 (默认6x6_250): ") or '6x6_250'
            generator.generate_random_markers(count, dict_type)
        
        elif choice == '4':
            ids_input = input("输入标记ID (用空格分隔，如: 0 1 2 3 4): ")
            marker_ids = [int(id_str) for id_str in ids_input.split()]
            dict_type = input("字典类型 (默认6x6_250): ") or '6x6_250'
            generator.create_marker_sheet(marker_ids, dict_type)
        
        elif choice == '5':
            generator.list_generated_markers()
        
        elif choice == '6':
            generator.cleanup_directory()
        
        elif choice == '7':
            print("退出程序")
            break
        
        else:
            print("无效选项，请重新选择")

if __name__ == "__main__":
    # 检查OpenCV是否安装
    try:
        import cv2
        import cv2.aruco
    except ImportError:
        print("错误: 需要安装OpenCV")
        print("请运行: pip install opencv-python opencv-contrib-python")
        exit(1)
    
    main()