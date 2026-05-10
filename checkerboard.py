from PIL import Image, ImageDraw

def generate_checkerboard(size=1080, squares=8):
    img = Image.new("RGB", (size, size), "black")
    draw = ImageDraw.Draw(img)
    
    square_size = size // squares
    
    for row in range(squares):
        for col in range(squares):
            if (row + col) % 2 == 1:
                top = row * square_size
                left = col * square_size
                bottom = top + square_size
                right = left + square_size
                draw.rectangle([left, top, right, bottom], fill="white")
                
    img.save("Final/images/checkerboard.png")

generate_checkerboard(1080, 8) 
