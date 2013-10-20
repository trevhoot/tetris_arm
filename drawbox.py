import sys
import pygame

#Initialize Everything
	pygame.init()
	self.screen = pygame.display.set_mode((self.width, self.height))
	pygame.display.set_caption('Tetris')
	pygame.mouse.set_visible(0)

#Create The Backgound
	background = pygame.Surface(self.screen.get_size())
	background = background.convert()
	background.fill((250, 250, 250))

#Display The Background
	self.screen.blit(background, (0, 0))
	pygame.display.flip()

'''def printBoard(self, board):
	for x in board.blocks:
		for y in board.blocks[x]:
			box = pygame.Rect(x,y,10,10)
			pygame.draw.rect(self.surf, (255,0,0), box, 0)
'''


