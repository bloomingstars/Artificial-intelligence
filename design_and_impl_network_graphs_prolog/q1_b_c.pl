edge(human,ako,creature).
edge(bird,ako,creature).
edge(man,ako,human).
edge(turkey,ako,bird).
edge(albert,isa,man).
edge(louis,isa,man).
edge(frank,isa,turkey).

/*multi-level ako*/
rel(X,ako,Y):-edge(X,ako,Z),rel(Z,ako,Y).
/*multi-level isa */
rel(X,isa,Y):-edge(X,isa,Z),rel(Z,isa,Y).
rel(X,ako,Y):-edge(X,ako,Y).
rel(X,isa,Y):-edge(X,isa,Y).
rel(X,isa,Y):-rel(X,ako,Y).

