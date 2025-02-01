import os
import argparse
from pillow_heif import open_heif
from PIL import Image

def convert_heic_to_best_format(heic_path):
    # Open the HEIC image
    heif_image = open_heif(heic_path)
    image = Image.frombytes(heif_image.mode, heif_image.size, heif_image.data, "raw", heif_image.mode, 0, 1)

    # Generate output paths
    base_name = os.path.splitext(heic_path)[0]
    jpg_path = f"{base_name}.jpg"
    png_path = f"{base_name}.png"

    # Save as JPG and PNG
    image.save(jpg_path, "JPEG", quality=85)
    image.save(png_path, "PNG", optimize=True)

    # Compare file sizes and keep the smaller one
    jpg_size = os.path.getsize(jpg_path)
    png_size = os.path.getsize(png_path)

    if jpg_size < png_size:
        os.remove(png_path)  # Keep JPG
        return jpg_path
    else:
        os.remove(jpg_path)  # Keep PNG
        return png_path

def process_directory(directory):
    for filename in os.listdir(directory):
        if filename.lower().endswith(".heic"):
            heic_path = os.path.join(directory, filename)
            new_file_path = convert_heic_to_best_format(heic_path)
            os.remove(heic_path)  # Remove the original HEIC file
            print(f"Converted and saved: {new_file_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert HEIC images to JPG or PNG (whichever is smaller).")
    parser.add_argument("--dir_name", required=True, help="Path to the directory containing HEIC images")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.dir_name):
        print("Error: Directory does not exist.")
    else:
        process_directory(args.dir_name)
