
#include <stdio.h>

// abstract type definition
typedef struct _Genotype_ *Genotype;

// set/get global number of genes
void genotype_set_size(int size);
int genotype_get_size();

// create new genotypes
Genotype genotype_create();

// release memory associated with g
void genotype_destroy(Genotype g);

// set/get fitness
void genotype_set_fitness(Genotype g, double fitness);
double genotype_get_fitness(Genotype g);
  
// get genes from genotype
const double *genotype_get_genes(Genotype g);

const double *genotype_get_node_types(Genotype g);

// read/write from stream
void genotype_fwrite(Genotype g, FILE *fd);
int genotype_fread(Genotype g, FILE *fd, int node_types_specified);

// print gentype
void print_genotype(Genotype g);

// random var from 0 to max
double drand();
