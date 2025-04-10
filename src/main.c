/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include "capteurs.h"

#define STACK_SIZE 2048

#define PRIORITY_task_ME 4
#define PRIORITY_task_ME_ecran 2
#define PRIORITY_task_set_Horodate 3
#define PRIORITY_task_update_capteur 1
#define PRIORITY_task_bluetooth 5

////////////////////////////////////////////////////////////////////////////////////
////						  Peripheral instantiation 							////
////////////////////////////////////////////////////////////////////////////////////

/* Definition des périphériques GPIO pour les LEDs */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

static const struct gpio_dt_spec led_0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led_2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led_3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);

/* Déclaration des capteurs */
const struct device *const hts221 = DEVICE_DT_GET_ANY(st_hts221);
const struct device *const lis2mdl = DEVICE_DT_GET_ANY(st_lis2mdl);
const struct device *const lis2dw12 = DEVICE_DT_GET_ANY(st_lis2dw12);
const struct device *const lsm6dso = DEVICE_DT_GET_ANY(st_lsm6dso);

/* Variables globales */
volatile int flag_transmition_bluetooth = 0;

/* Sémaphores */
struct k_sem semaphore_I2C;
struct k_sem semaphore_ecriture_lecture;
struct k_sem semaphore_task_set_Horodate;
struct k_sem semaphore_ME_ecran;
struct k_sem semaphore_update_capteur;
struct k_sem semaphore_bluetooth;

/* Stacks et threads */
K_THREAD_STACK_DEFINE(thread_stack_area_update_capteur, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_stack_area_bluetooth, STACK_SIZE);
static struct k_thread thread_task_update_capteur, thread_task_bluetooth;

////////////////////////////////////////////////////////////////////////////////////
////						   Fonction Bluetooth									////
////////////////////////////////////////////////////////////////////////////////////

void init_bluetooth(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");
}

////////////////////////////////////////////////////////////////////////////////////
////						   Tâches												////
////////////////////////////////////////////////////////////////////////////////////

/* Tâche pour mettre à jour les capteurs */
void task_update_capteur()
{
    while (1) {
        k_sem_take(&semaphore_update_capteur, K_FOREVER);

        k_sem_take(&semaphore_I2C, K_FOREVER);
        k_sem_take(&semaphore_ecriture_lecture, K_FOREVER);

        if (sensor_sample_fetch(hts221) == 0) {
            struct sensor_value temp, hum;
            sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);

            printk("HTS221: Temperature: %.1f C, Humidity: %.1f%%\n",
                   sensor_value_to_double(&temp), sensor_value_to_double(&hum));
        } else {
            printk("Failed to fetch data from HTS221\n");
        }

        k_sem_give(&semaphore_I2C);
        k_sem_give(&semaphore_ecriture_lecture);

        k_yield();
    }
}

/* Tâche Bluetooth */
void task_bluetooth()
{
    init_bluetooth();

    while (1) {
        k_sem_take(&semaphore_bluetooth, K_FOREVER);

        if (flag_transmition_bluetooth == 1) {
            k_sem_take(&semaphore_ecriture_lecture, K_FOREVER);

            printk("Bluetooth transmission - Data sent\n");

            k_sem_give(&semaphore_ecriture_lecture);

            if (flag_transmition_bluetooth == 1) {
                k_sem_give(&semaphore_bluetooth);
            }

            k_yield();
        } else {
            printk("Bluetooth task yielding to scheduler.\n");
            k_yield();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////
////						   Fonction principale								////
////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
    printk("Starting application\n");

    // Initialisation des LEDs
    if (!gpio_is_ready_dt(&led_0) || gpio_pin_configure_dt(&led_0, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Failed to initialize LED 0\n");
        return 0;
    }
    if (!gpio_is_ready_dt(&led_1) || gpio_pin_configure_dt(&led_1, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Failed to initialize LED 1\n");
        return 0;
    }
    if (!gpio_is_ready_dt(&led_2) || gpio_pin_configure_dt(&led_2, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Failed to initialize LED 2\n");
        return 0;
    }
    if (!gpio_is_ready_dt(&led_3) || gpio_pin_configure_dt(&led_3, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Failed to initialize LED 3\n");
        return 0;
    }

    // Vérification des capteurs
    if (!device_is_ready(hts221)) {
        printk("HTS221: device not ready. Check I2C connection or device tree configuration.\n");
        return 0;
    }
    if (!device_is_ready(lis2mdl)) {
        printk("%s: device not ready.\n", lis2mdl->name);
        return 0;
    }
    if (!device_is_ready(lis2dw12)) {
        printk("%s: device not ready.\n", lis2dw12->name);
        return 0;
    }
    if (!device_is_ready(lsm6dso)) {
        printk("LSM6DSO: device not ready. Check I2C connection or device tree configuration.\n");
        return 0;
    }

    // Configuration des capteurs
    lis2mdl_config(lis2mdl);
    lis2dw12_config(lis2dw12);
    lsm6dso_config(lsm6dso);

    // Initialisation des sémaphores
    k_sem_init(&semaphore_I2C, 1, 1);
    k_sem_init(&semaphore_ecriture_lecture, 1, 1);
    k_sem_init(&semaphore_task_set_Horodate, 1, 1);
    k_sem_init(&semaphore_ME_ecran, 0, 1);
    k_sem_init(&semaphore_update_capteur, 0, 1);
    k_sem_init(&semaphore_bluetooth, 0, 1);

    // Test des capteurs
    k_sem_give(&semaphore_update_capteur);

    // Création des tâches
    k_thread_create(&thread_task_update_capteur, thread_stack_area_update_capteur,
                    K_THREAD_STACK_SIZEOF(thread_stack_area_update_capteur),
                    task_update_capteur,
                    NULL, NULL, NULL,
                    PRIORITY_task_update_capteur, 0, K_NO_WAIT);

    k_thread_create(&thread_task_bluetooth, thread_stack_area_bluetooth,
                    K_THREAD_STACK_SIZEOF(thread_stack_area_bluetooth),
                    task_bluetooth,
                    NULL, NULL, NULL,
                    PRIORITY_task_bluetooth, 0, K_NO_WAIT);

    // Démarrage des tâches
    k_thread_start(&thread_task_update_capteur);
    k_thread_start(&thread_task_bluetooth);

    return 0;
}