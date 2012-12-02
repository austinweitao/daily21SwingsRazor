function crc8(data,datalen)
{
  var i,j;
  var c;
  var CRC=0x00;
  for (j=0; j<datalen; j++)
  {
    c = data[j];
    for(i = 0; i<8; i++){
        if(((CRC ^ c) & 0x01)==0){
            CRC >>= 1;
        }
        else{
            CRC ^= 0x18;
            CRC >>= 1;
            CRC |= 0x80;
        }
        c >>= 1;
    }
  }
  if(CRC == 254 || CRC == 255 || CRC == 2) {
    CRC = 99;
  }
  return CRC;
}

function list(a)       
{
  outlet(0,crc8(arguments,arguments.length));
}
