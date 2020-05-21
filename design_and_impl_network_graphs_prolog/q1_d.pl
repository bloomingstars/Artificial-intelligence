edge(human,ako,creature).
edge(bird,ako,creature).
edge(man,ako,human).
edge(turkey,ako,bird).
edge(albert,isa,man).
edge(louis,isa,man).
edge(frank,isa,turkey).

property(human, legs, two).
property(bird,fly,yes).
property(louis,legs,one).
property(turkey,fly,no).

rel(X,ako,Y):-edge(X,ako,Z),rel(Z,ako,Y).
rel(X,isa,Y):-edge(X,isa,Z),rel(Z,isa,Y).
rel(X,ako,Y):-edge(X,ako,Y).
rel(X,isa,Y):-edge(X,isa,Y).
rel(X,isa,Y):-rel(X,ako,Y).