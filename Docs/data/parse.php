<?php

$json = file_get_contents("msg.json");
$arr = json_decode($json);

$table = [];
foreach($arr as $msg) {
  $msg = (array)$msg;
  if ($msg['port'] == 68 && $msg['dev_id'] == 'ondrej_hruska') {
    $table[] = [
      'counter' => $msg['counter'],
      'date' => ((array)$msg['metadata'])['time'],
      'payload_raw' => $msg['payload_raw'],
      'payload' => parsePayload($msg['payload_raw']),
    ];
  }
}

print_r($table);

$fil = "Counter\tDate\tTemperature [°C]\tHumidity [%RH]\tPressure [hPa]\tGas_R [Ohm]\n";
foreach($table as $row) {
  $p = $row['payload'];
  $fil .= "$row[counter]\t$row[date]\t$p[temp]\t$p[hum]\t$p[press]\t$p[gas_r]\n";
}

file_put_contents("msg.csv", $fil);







function parsePayload($pl64) {
  $pl = base64_decode($pl64);
  
  /*  
	pb_i16(&pb, voc_data.temperature); // Cx100
	pb_u16(&pb, (uint16_t) (voc_data.humidity / 10)); // discard one place -> %x100
	pb_u16(&pb, (uint16_t) (voc_data.pressure - 85000)); // send offset from 850 hPa -> Pa
	pb_u32(&pb, (uint16_t) (voc_data.gas_resistance)); // ohms, full size
	*/
  $res = [];
  $t = (ord($pl[0])<<8 | ord($pl[1]));
  if ($t&0x8000) $t = ~$t - 1;
  $res['temp'] = $t/100;
  
  $t = (ord($pl[2])<<8 | ord($pl[3]));
  $res['hum'] = $t/100;
  
  $t = (ord($pl[4])<<8 | ord($pl[5]));
  $res['press'] = (85000+$t)/100;
  
  $t = (ord($pl[6])<<24 | ord($pl[7])<<16 | ord($pl[8])<<8 | ord($pl[9]));
  $res['gas_r'] = $t;
  
  return $res;
}












