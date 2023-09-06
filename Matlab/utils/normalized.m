function normalized_data = normalized(input,max)
originalData = input;
originalData = int32(originalData);
originalData(originalData==0)=1;
originalData(originalData>max)=max;
normalized_data = originalData;