#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "string"
#include "vector"

using namespace std;

// reads the first N unsigned integers from an array, where they are
// separated by colons and ultimately terminated with _, \r, or 0.
// modifies the array temporarily but fixes it by return time
// drops -2 into the array where numbers could not be read due to syntax issues
// and -20 where the numbers can't be read because there weren't that many
// numbers
static void read_uints_from_array(char * start, int * output_array, int N){
  int chan = 0;
  char * begin_this_num = start;
  char * end_this_num = start;
  bool done = false;
  // step through each number
  while (chan < N && !done){
    // find end of this number
    while (*end_this_num != ':' && *end_this_num != '\r' && *end_this_num != '_' && *end_this_num != 0){
      end_this_num++;
    }
    // if we hit sometihng that wasn't a colon, no channels left after this number
    if (*end_this_num != ':')
      done = true;
    
    // null-terminate the number for a second, but be ready to fix it
    char old = *end_this_num;
    *end_this_num = 0;

    // and parse it into an integer 
    output_array[chan] = strtol(begin_this_num, NULL, 10);
    
    // move onto next number after repairing this one
    *end_this_num = old;
    chan++;
    end_this_num++;
    begin_this_num = end_this_num;
  }

  while (chan < N){
    output_array[chan] = -20;
    chan++;
  }

}

int main(){
  std::vector<string> inputs;
  std::vector<int> input_sizes;
  std::vector<std::vector<int> > outputs;

  inputs.push_back("0");
  input_sizes.push_back(1);
  outputs.push_back(std::vector<int>({0}));
  inputs.push_back("0\r");
  input_sizes.push_back(1);
  outputs.push_back(std::vector<int>({0}));
  inputs.push_back("0_");
  input_sizes.push_back(1);
  outputs.push_back(std::vector<int>({0}));

  inputs.push_back("_");
  input_sizes.push_back(0);
  outputs.push_back(std::vector<int>({}));


  inputs.push_back("123:456:789\r");
  input_sizes.push_back(0);
  outputs.push_back(std::vector<int>({123, 456, 789}));


  for (int i=0; i<inputs.size(); i++){
    int * output_array = (int *) malloc(sizeof(int) * input_sizes[i]);
    char * input_array = (char *) malloc(sizeof(char) * inputs[i].size()+1);
    memcpy(input_array, inputs[i].c_str(), sizeof(char) * inputs[i].size()+1);
    read_uints_from_array(input_array, output_array, input_sizes[i]);
    printf("Read %d... ", i);

    for (int j=0; j<input_sizes[i]; j++){
      if (output_array[j] != outputs[i][j]){
        printf("* term %d: %d!=%d *", j, output_array[j], outputs[i][j]);
      }
    } 
    printf("\n");

    free(output_array);
    free(input_array);
  }

  return 0;
}