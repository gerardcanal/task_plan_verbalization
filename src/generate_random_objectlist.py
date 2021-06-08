import random
N = 100
def list_to_yaml(list):
	for a in list:
		x = "{:.3f}".format(random.random() * N)
		y = "{:.3f}".format(random.random() * N)
		z = "{:.3f}".format(random.random() * N)
		print(a + ': "(' + x + ', ' + y + ', ' + x +')"'  )