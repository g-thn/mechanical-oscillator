

class pieces:
    couleur = "blanc"

class Maison:
    #def __init__(self, nb_pieces, nb_portes, nb_fenetres):
    nb_pieces = 2
    nb_portes = 5
    nb_fenetres = 10
    piece1 = pieces()
    piece2 = pieces()


maison1 = Maison()
maison2 = Maison()
print(maison1.nb_pieces)
maison1.nb_pieces = 4
print(maison1.nb_pieces)
print(maison1.piece1.couleur)