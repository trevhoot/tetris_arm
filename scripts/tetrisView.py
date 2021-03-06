import sys
import pygame
import basicTetris

class Display():
	def __init__(self):	
		self.width = 500
		self.height = 800
		self.blockSide = 20
		self.color = (255,0,0) #Change for each piece that comes in
		
	#Initialize Everything
		pygame.init()
		self.screen = pygame.display.set_mode((self.width, self.height))
		pygame.display.set_caption('Tetris')
		pygame.mouse.set_visible(0)
	
	#Create The Backgound
		background = pygame.Surface(self.screen.get_size())
		self.bg = background.convert()
		self.bg.fill((255, 255, 255))
	
	#Put Text On The Background, Centered
		if pygame.font:
			font = pygame.font.Font(None, 36)
			text = font.render("Play Tetris!", 1, (10, 10, 10))
			textpos = text.get_rect(centerx=self.bg.get_width()/2)
			self.bg.blit(text, textpos)

	#Display The Background
		self.screen.blit(self.bg, (0, 0))
		pygame.display.flip()
	
	def scaleBlock(self, x,y):
		box = pygame.Rect(self.width*x/11., self.height *(1-y/20.), self.blockSide, self.blockSide)
		return box

	def printBoard(self, board):
		self.bg.lock()
		for x in board.blocks:
			for y in board.blocks[x]:
				box = self.scaleBlock(x+1,y+1)
				pygame.draw.rect(self.screen, self.color, box, 0)
		self.bg.unlock()
		pygame.display.update()

def main():
#Prepare Game Objects
	clock = pygame.time.Clock()
	board = basicTetris.Board()
	d = Display()

#Main Loop
	while 1:
		#wait = raw_input("wait")
		clock.tick(60)
		for event in pygame.event.get(): 
			if event.type == pygame.QUIT: 
				sys.exit(0)
			if event.type == pygame.MOUSEBUTTONDOWN:
				board.newPiece()
				board.choosePlace()
			else: 
				pass

		d.printBoard(board)

#this calls the 'main' function when this script is executed
if __name__ == '__main__': main()
