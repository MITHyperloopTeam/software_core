#include "arduino_matrix_math.h"
#include "stdio.h"

void printMatrix(char * str, XenMatrix::Mat3<float> m){
  printf("%s:", str);
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      printf("%f, ", m(i, j));
    }
  }
  printf("\n");
}

int main(){
  XenMatrix::Mat3<float> count;
  count(0, 0) = 1.0;
  count(0, 1) = 2.0;
  count(0, 2) = 3.0;
  count(1, 0) = 4.0;
  count(1, 1) = 5.0;
  count(1, 2) = 6.0;
  count(2, 0) = 7.0;
  count(2, 1) = 8.0;
  count(2, 2) = 9.0;
  
  XenMatrix::Vec3<float> one_two_three(0.0, 1.0, 2.0);
  
  XenMatrix::Vec3<float> out = count * one_two_three;
  printf("%f, %f, %f\n", out.x(), out.y(), out.z());

  XenMatrix::Mat3<float> countOtherWay;
  countOtherWay(0, 0) = 1.0;
  countOtherWay(1, 0) = 2.0;
  countOtherWay(2, 0) = 3.0;
  countOtherWay(0, 1) = 4.0;
  countOtherWay(1, 1) = 5.0;
  countOtherWay(2, 1) = 6.0;
  countOtherWay(0, 2) = 7.0;
  countOtherWay(1, 2) = 8.0;
  countOtherWay(2, 2) = 9.0;

  XenMatrix::Mat3<float> twos;
  twos(0, 0) = 2.0;
  twos(0, 1) = 2.0;
  twos(0, 2) = 2.0;
  twos(1, 0) = 2.0;
  twos(1, 1) = 2.0;
  twos(1, 2) = 2.0;
  twos(2, 0) = 2.0;
  twos(2, 1) = 2.0;
  twos(2, 2) = 2.0;

  XenMatrix::Mat3<float> result = count * countOtherWay * count.transpose() + twos;
  printMatrix("count*twos*count.' + twos", result);

  if (result(0, 0) == 230 &&
      result(0, 1) == 518 &&
      result(0, 2) == 806 &&
      result(1, 0) == 554 &&
      result(1, 1) == 1247 &&
      result(1, 2) == 1940 &&
      result(2, 0) == 878 &&
      result(2, 1) == 1976 &&
      result(2, 2) == 3074){
    printf("Success!\n");
} else {
    printf("FAILURE!\n");
}

}