/* Copyright (c) 2009, Armin Biere, Johannes Kepler University. */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/times.h>
#include <unistd.h>

#define MAX 20
static int clause[MAX + 1];

static int pick(int from, int to) {
  assert(from <= to);
  return (rand() % (to - from + 1)) + from;
}

int main(int argc, char **argv) {
  int seed, nlayers, **layers, *width, *low, *high, *clauses;
  int i, j, k, l, m, n, o, p, sign, lit, layer, w;
  int **unused, *nunused;
  char *mark;
  int x = 0, y = 0, z = 0, t = 0, addedClauseXor = 0;

  if (argc > 1 && !strcmp(argv[1], "-h")) {
    printf("usage: cnfuzz [-h][<seed>]\n"
           "\n"
           "If the seed is not specified it is calculated from the process id\n"
           "and the current system time (in seconds).\n");
    return 0;
  }

  seed = (argc > 1) ? atoi(argv[1]) : abs((times(0) * getpid()) >> 1);
  printf("c seed %d\n", seed);
  srand(seed);
  w = pick(10, 70);
  printf("c max width %d\n", w);
  nlayers = pick(1, 20);
  printf("c layers %d\n", nlayers);
  layers = calloc(nlayers, sizeof *layers);
  width = calloc(nlayers, sizeof *width);
  low = calloc(nlayers, sizeof *low);
  high = calloc(nlayers, sizeof *high);
  clauses = calloc(nlayers, sizeof *clauses);
  unused = calloc(nlayers, sizeof *unused);
  nunused = calloc(nlayers, sizeof *nunused);
  for (i = 0; i < nlayers; i++) {
    width[i] = pick(10, w);
    low[i] = i ? high[i - 1] + 1 : 1;
    high[i] = low[i] + width[i] - 1;
    m = width[i];
    if (i)
      m += width[i - 1];
    n = (pick(300, 450) * m) / 100;
    clauses[i] = n;
    printf("c layer[%d] = [%d..%d] w=%d v=%d c=%d r=%.2f\n", i, low[i], high[i],
           width[i], m, n, n / (double)m);

    nunused[i] = 2 * (high[i] - low[i] + 1);
    unused[i] = calloc(nunused[i], sizeof *unused[i]);
    k = 0;
    for (j = low[i]; j <= high[i]; j++)
      for (sign = -1; sign <= 1; sign += 2)
        unused[i][k++] = sign * j;
    assert(k == nunused[i]);
  }
  n = 0;
  m = high[nlayers - 1];
  mark = calloc(m + 1, 1);
  for (i = 0; i < nlayers; i++)
    n += clauses[i];

  // add xor clauses.
  int vars[m];
  for (i = 0; i < m; i++)
    vars[i] = i + 1;

  // shuffle
  for (i = 0; i < m; i++) {
    x = rand() % m;
    y = rand() % m;
    z = vars[x];
    vars[x] = vars[y];
    vars[y] = z;
  }

  int nbRun = (m < 30) ? 1 : 3;
  int pos = 0;

  printf("p cnf %d %d\n", m, n + (nbRun * 12));

  while (nbRun--) {
    if (pos + 3 < m) {
      addedClauseXor += 4;
      x = vars[pos++];
      y = vars[pos++];
      z = vars[pos++];

      if (rand() % 2)
        x = -x;
      if (rand() % 2)
        y = -y;
      if (rand() % 2)
        z = -z;

      printf("%d %d %d 0\n", -x, y, z);
      printf("%d %d %d 0\n", x, -y, z);
      printf("%d %d %d 0\n", x, y, -z);
      printf("%d %d %d 0\n", -x, -y, -z);
    }

    pos -= rand() % 3;

    if (pos + 4 < m) {
      x = vars[pos++];
      y = vars[pos++];
      z = vars[pos++];
      t = vars[pos++];

      if (rand() % 2)
        x = -x;
      if (rand() % 2)
        y = -y;
      if (rand() % 2)
        z = -z;
      if (rand() % 2)
        t = -t;

      addedClauseXor += 8;
      printf("%d %d %d %d 0\n", -x, y, z, t);
      printf("%d %d %d %d 0\n", x, -y, z, t);
      printf("%d %d %d %d 0\n", x, y, -z, t);
      printf("%d %d %d %d 0\n", x, y, z, -t);
      printf("%d %d %d %d 0\n", -x, -y, -z, t);
      printf("%d %d %d %d 0\n", -x, -y, z, -t);
      printf("%d %d %d %d 0\n", -x, y, -z, -t);
      printf("%d %d %d %d 0\n", x, -y, -z, -t);
    }
  }

  for (i = 0; i < nlayers; i++) {
    for (j = 0; j < clauses[i]; j++) {
      l = 3;
      while (l < MAX && pick(17, 19) != 17)
        l++;

      for (k = 0; k < l; k++) {
        layer = i;
        while (layer && pick(3, 4) == 3)
          layer--;
        if (nunused[layer]) {
          o = nunused[layer] - 1;
          p = pick(0, o);
          lit = unused[layer][p];
          if (mark[abs(lit)])
            continue;
          nunused[layer] = o;
          if (p != o)
            unused[layer][p] = unused[layer][o];
        } else {
          lit = pick(low[layer], high[layer]);
          if (mark[lit])
            continue;
          sign = (pick(31, 32) == 31) ? 1 : -1;
          lit *= sign;
        }
        clause[k] = lit;
        mark[abs(lit)] = 1;
        printf("%d ", lit);
      }
      printf("0\n");
      for (k = 0; k < l; k++)
        mark[abs(clause[k])] = 0;
    }
  }
  free(mark);
  free(clauses);
  free(high);
  free(low);
  free(width);
  free(nunused);
  for (i = 0; i < nlayers; i++)
    free(layers[i]), free(unused[i]);
  free(layers);
  return 0;
}
