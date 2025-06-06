/* Generated by Edge Impulse
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Generated on: 15.12.2022 22:59:14

#ifndef trained_model_GEN_H
#define trained_model_GEN_H

#include "edge-impulse-sdk/tensorflow/lite/c/common.h"

// Sets up the model with init and prepare steps.
TfLiteStatus trained_model_init( void*(*alloc_fnc)(size_t,size_t) );
// Returns the input tensor with the given index.
TfLiteTensor *trained_model_input(int index);
// Returns the output tensor with the given index.
TfLiteTensor *trained_model_output(int index);
// Runs inference for the model.
TfLiteStatus trained_model_invoke();
//Frees memory allocated
TfLiteStatus trained_model_reset( void (*free)(void* ptr) );


// Returns the number of input tensors.
inline size_t trained_model_inputs() {
  return 1;
}
// Returns the number of output tensors.
inline size_t trained_model_outputs() {
  return 1;
}

inline void *trained_model_input_ptr(int index) {
  return trained_model_input(index)->data.data;
}
inline size_t trained_model_input_size(int index) {
  return trained_model_input(index)->bytes;
}
inline int trained_model_input_dims_len(int index) {
  return trained_model_input(index)->dims->data[0];
}
inline int *trained_model_input_dims(int index) {
  return &trained_model_input(index)->dims->data[1];
}

inline void *trained_model_output_ptr(int index) {
  return trained_model_output(index)->data.data;
}
inline size_t trained_model_output_size(int index) {
  return trained_model_output(index)->bytes;
}
inline int trained_model_output_dims_len(int index) {
  return trained_model_output(index)->dims->data[0];
}
inline int *trained_model_output_dims(int index) {
  return &trained_model_output(index)->dims->data[1];
}

#endif
