import io
import os
from PIL import Image, ImageDraw

def load_icon_byte_arr(file_name, white_background=False, overlay=False):
    dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../assets')
    asset_path = os.path.join(dir_path, file_name)
    img = Image.new('RGB', (120, 120), color='black')
    if white_background:
        img_back = ImageDraw.Draw(img)
        img_back.rectangle([30,45,90,75], fill="white", outline="white") 
    icon = Image.open(asset_path).resize((80, 80))
    img.paste(icon, (20, 20), icon)

    if overlay:
        img = img.convert('RGBA')
        tint_color = (0, 0, 0)
        transparency = 0.5
        opacity = int(255 * transparency)
        overlay = Image.new('RGBA', (120, 120), tint_color+(0,))
        draw = ImageDraw.Draw(overlay)
        draw.rectangle([0,0,120,120], fill=tint_color+(opacity,))

        img = Image.alpha_composite(img, overlay)
        img = img.convert('RGB')

    img_byte_arr = io.BytesIO()
    img.save(img_byte_arr, format='JPEG')
    img_byte_value = img_byte_arr.getvalue()

    return img_byte_value

def load_icon(file_name, white_background=False, overlay=False):
    dir_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../assets')
    asset_path = os.path.join(dir_path, file_name)
    img = Image.new('RGB', (80, 80), color='black')
    if white_background:
        img_back = ImageDraw.Draw(img)
        img_back.rectangle([30,45,90,75], fill="white", outline="white") 
    icon = Image.open(asset_path).resize((80, 80))
    img.paste(icon, (0, 0), icon)
    img = img.convert('RGBA')

    if overlay:
        tint_color = (0, 0, 0)
        transparency = 0.5
        opacity = int(255 * transparency)
        overlay = Image.new('RGBA', (80, 80), tint_color+(0,))
        draw = ImageDraw.Draw(overlay)
        draw.rectangle([0,0,80,80], fill=tint_color+(opacity,))

        img = Image.alpha_composite(img, overlay)
        # img = img.convert('RGB')
    return img