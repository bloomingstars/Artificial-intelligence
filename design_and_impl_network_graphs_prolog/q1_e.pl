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

prop(X,fly,yes):- property(X,fly,yes).
prop(X,fly,yes):-property(Z,fly,yes),rel(X,isa,Z),not(property(X,fly,no)), not(rel(X,isa,Y)),not(property(Y,fly,no)).

prop(X,legs,two):- property(X,legs,two).
prop(X,legs,two):-property(Z,legs,two),rel(X,isa,Z),not((property(X,legs,Y),not(Y=two))).

prop(X,legs,one):- property(X,legs,one).
prop(X,legs,one):-property(Z,legs,one),rel(X,isa,Z),not((property(X,legs,Y),not(Y=one))).
