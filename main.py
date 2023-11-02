import pygame as pg

width = 1280
height = 720


pg.init()

screen = pg.display.set_mode((width, height))
pg.display.set_caption('My game')

running = True
while running:
    
    pg.display.update()
    
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_ESCAPE:
                pg.quit()
