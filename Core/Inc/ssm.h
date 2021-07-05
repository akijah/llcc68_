/*
 * ssm.h
 *
 *  Created on: Jul 5, 2021
 *      Author: AKI
 */

#ifndef INC_SSM_H_
#define INC_SSM_H_

#ifdef __cplusplus
 extern "C" {
#endif
#define S_TYPE		0xAB  //Sensor type
#define S_VERSION	0x01  //Protocol version
#define SERIAL_SIZE 8
#define PACKET_SIZE 32
#define SUF_WL				0xAB	//признак датчика WL
enum sensorStates
{
 ssCheckSerial,     // 1. Проверяем серийный номер. Если присутствует, то 4.
 ssButtonWait,      // 2. Ждем команды пользователя (нажатие на кнопку).
 ssGenerateSerial,  // 3. Если серийного номера нет, генерируем его.
 ssBlinkHaveNumber, // 4. Подаем сигнал (моргаем диодом) о наличии номера.
 ssRegister,        // 5. Пытаемся зарегистрироваться у панели. Если не получается, то 2.
 ssBlinkRegistered, // 6. Подаем сигнал (моргаем диодом) об успешной регистрации.
 ssNormalMode,      // 7. Обмениваемся данными с панелью. Если успех, то 7.
 ssSeldomMode       // 8. Обмениваемся данными с панелью редко. Если передача наладилась, то 7, иначе 8.

//Нажатие на кнопку: переходим к 4.
//Длительное нажатие на кнопку: генерируем серийный номер и переходим к 4.
};

void SS_Init(void);
void SS_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SSM_H_ */
