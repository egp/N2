// BinaryOutput.h v1
#ifndef BINARY_OUTPUT_H
#define BINARY_OUTPUT_H

class IBinaryOutput {
public:
  virtual ~IBinaryOutput() {}
  virtual void setOn(bool on) = 0;
};

#endif
// BinaryOutput.h v1