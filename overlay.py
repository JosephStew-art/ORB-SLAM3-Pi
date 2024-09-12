from PIL import Image
import numpy as np
from pdf2image import convert_from_path
import os

def pdf_to_png(pdf_path):
    pages = convert_from_path(pdf_path, 300)  # 300 DPI
    png_path = pdf_path.rsplit('.', 1)[0] + '.png'
    pages[0].save(png_path, 'PNG')
    return png_path

def extract_red_line(image_path):
    # Open the image
    img = Image.open(image_path).convert('RGBA')
    
    # Convert to numpy array
    img_array = np.array(img)
    
    # Create a mask for red pixels
    red_mask = (img_array[:,:,0] > 200) & (img_array[:,:,1] < 100) & (img_array[:,:,2] < 100)
    
    # Create a new image with only the red line
    red_line = np.zeros_like(img_array)
    red_line[red_mask] = [255, 0, 0, 255]  # Set alpha to 255 for red pixels
    
    return Image.fromarray(red_line, 'RGBA')

def overlay_images(base_image_path, overlay_image_path, output_path):
    # Convert PDFs to PNGs if necessary
    if base_image_path.lower().endswith('.pdf'):
        base_image_path = pdf_to_png(base_image_path)
    if overlay_image_path.lower().endswith('.pdf'):
        overlay_image_path = pdf_to_png(overlay_image_path)

    # Open the base image
    base_img = Image.open(base_image_path).convert('RGBA')
    
    # Extract the red line from the overlay image
    red_line = extract_red_line(overlay_image_path)
    
    # Resize red_line if sizes don't match
    if base_img.size != red_line.size:
        red_line = red_line.resize(base_img.size, Image.LANCZOS)
    
    # Overlay the red line on the base image
    result = Image.alpha_composite(base_img, red_line)
    
    # Save the result
    result.save(output_path)

# Usage
base_image_path = '/home/joseph/Desktop/SLAM/ORB-SLAM3-Pi/Outputs/MH01 v1/MH01_mono_pi.pdf'
overlay_image_path = '/home/joseph/Desktop/SLAM/ORB-SLAM3-Pi/Outputs/MH01 v1/MH01_mono.pdf'
output_path = '/home/joseph/Desktop/SLAM/ORB-SLAM3-Pi/Outputs/MH01 v1/combined_plot.png'

overlay_images(base_image_path, overlay_image_path, output_path)