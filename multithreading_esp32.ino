/*
 *  Copyright(C) 2018 by Yukiya Ishioka
 *  https://qiita.com/Yukiya_Ishioka/items/ae21b4a1aecf69a885f2
 */

#include <freertos/FreeRTOS.h>    /* FreeRTOSを用いるためのヘッダファイル */

#define  LED1PIN   22    /* LED1のポート番号 */
#define  LED2PIN   23    /* LED2のポート番号 */

void  task1( void *param )
{
  while( 1 ) {
    vTaskDelay(500);
    digitalWrite(LED1PIN, HIGH);  /* LED on */
    vTaskDelay(500);
    digitalWrite(LED1PIN, LOW);   /* LED off */
  }
}

void  task2( void *param )
{
  while( 1 ) {
    vTaskDelay(200);
    digitalWrite(LED2PIN, HIGH);  /* LED on */
    vTaskDelay(200);
    digitalWrite(LED2PIN, LOW);   /* LED off */
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.print("setup()\n");

  pinMode(LED1PIN, OUTPUT);
  pinMode(LED2PIN, OUTPUT);
  digitalWrite(LED1PIN, LOW);  /* LED1 off */
  digitalWrite(LED2PIN, LOW);  /* LED2 off */

  /* create task */
  xTaskCreatePinnedToCore( task1,   /* タスクの入口となる関数名 */
                           "TASK1", /* タスクの名称 */
                           0x800,   /* スタックサイズ */
                           NULL,    /* パラメータのポインタ */
                           1,       /* プライオリティ */
                           NULL,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */

  xTaskCreatePinnedToCore( task2,
                           "TASK2",
                           0x800,
                           NULL,
                           2,
                           NULL,
                           0 );
}

void loop()
{
  Serial.print("loop()\n");
  vTaskDelay(1000);
}
