#include <iostream>

void convolute(double in[], double kernel[], int nIn, int nKernel) {
  
  double result[nIn + nKernel - 1];

  for (int ii=0; ii < nIn + nKernel - 1; ii++) {
    result[ii] = 0;
    int jMin = (ii >= nKernel - 1) ? ii - (nKernel - 1) : 0;
    int jMax = (ii < nIn - 1) ? ii : nIn - 1;

    for (int j = jMin; j <= jMax; j++) {
      result[ii] += in[j] * kernel[ii - j];
    }
  }

  std::cout << "\n\n";
    for(int ii=0; ii < nIn+nKernel - 1; ii++) {
      std::cout << result[ii] << ", ";
    }
    std::cout << "\n\n";

  // assign back to original
  // todo: make this cleaner
  if (nIn % 2) {
    int middleOut = (nIn - 1) / 2;
    int middleResult = ( nKernel + nIn - 1 - 1 ) / 2;

    in[middleOut] = result[middleResult];
    for (int ii=1; ii <= middleOut; ii++) {
      in[middleOut + ii] = result[middleResult + ii];
      in[middleOut - ii] = result[middleResult - ii];
    }
  }
  else {
    int middleOut = nIn / 2;
    int middleResult = ( nKernel + nIn - 1 ) / 2;

    for (int ii=0; ii < middleOut; ii++) {
      in[middleOut + ii] = result[middleResult + ii];
      in[middleOut - 1 - ii] = result[middleResult - 1 - ii];
    }
  }
}

int main() {
  double in[3];
  double kernel[3];

  for (int i=0;i<3;i++){
    in[i] = 1.0;
  }
  for (int i=0; i<3;i++){
    kernel[i] = 1.0;
  }

  convolute(in, kernel, 3, 3);

  for(int i=0;i<3;i++){
    std::cout << in[i] << std::endl;
  }
}