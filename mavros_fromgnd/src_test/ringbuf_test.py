from ringbuf import RingBuffer

if __name__=="__main__":
  buff = RingBuffer(50)

  temp = str("123456789")

  # print("temp size : " + str(len(temp)))
  # print(temp[:])
  for i in range(10,40):
    buff.push(chr(i))
    
  for i in range(10,40):
    test = buff.pop()
    print("Pop : " + test + "is Oct" + " is a type, " + str(type(test)))
