from PIL import Image, ImageDraw, ImageFilter

rect_w = 2592
rect_h = 1944
source_img = Image.new('L', (rect_w, rect_h))
draw = ImageDraw.Draw(source_img)
draw.rectangle(((0, 0), (rect_w, rect_h)), fill="black")
box_w = 50
dot_dia = 12
n_box = 26
start_x1 = 500
start_y1 = 300
draw.rectangle(((start_x1-50, start_y1-50), (start_x1+n_box*box_w+50, start_y1+n_box*box_w+50)), fill="white")

for i in range(0, n_box):
    for j in range(0, n_box):
        # x1 = start_x1 + i*box_w
        # y1 = start_y1 + j*box_w
        x1 = start_x1 + i*box_w
        y1 = start_y1 + j*box_w
        draw.ellipse([x1, y1, x1+dot_dia, y1+dot_dia], fill="black", outline="black")
        # if ((i%2 == 0 and j%2 == 1) or (i%2 == 1 and j%2 == 0)):
        #     draw.rectangle([x1, y1, x1+box_w, y1+box_w], fill="black", outline="black")

blurred_image = source_img.filter(ImageFilter.GaussianBlur(radius=2))
width, height = blurred_image.size
m = -0.3
xshift = abs(m) * width
new_width = width + int(round(xshift))
blurred_image = blurred_image.transform((new_width, height), Image.AFFINE,
        (1, m, -xshift if m > 0 else 0, 0, 1, 0), Image.BICUBIC)
print(blurred_image.size)
# blurred_image.save("checkerboard.png", "PNG")
blurred_image.save("circleboard.png", "PNG")