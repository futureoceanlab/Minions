from PIL import Image, ImageDraw

# seed the pseudorandom number generator
from random import seed
from random import random
# seed random number generator
seed(1)
# seed the pseudorandom number generator
from random import seed
from random import random
# seed random number generator
seed(2)
rect_w = 2000
rect_h = 2000
source_img = Image.new('RGBA', (rect_w, rect_h))
draw = ImageDraw.Draw(source_img)
draw.rectangle(((0, 0), (rect_w, rect_h)), fill="black")
outline = 3 # line thickness
circle_dia = 40
for i in range(1, 500):
    x1 = round(random()*rect_w)
    y1 = round(random()*rect_w)
    draw.ellipse((x1-outline, y1-outline, x1+circle_dia+outline, y1+circle_dia+outline), 
                fill = 'white', outline ='white')
    draw.ellipse((x1, y1, x1+circle_dia, y1+circle_dia), fill = 'black', outline ='white')

source_img.save("pattern.png", "PNG")