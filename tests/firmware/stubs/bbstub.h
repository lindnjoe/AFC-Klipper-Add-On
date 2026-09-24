/* Stubs so wifiportal.c's parsing and save state machine can be compiled and
 * driven on a workstation. Nothing here pretends to be lwIP or the CYW43 --
 * these are the smallest declarations that let the real file compile, plus
 * recording versions of the few calls a test needs to observe (tcp_write). */
#ifndef BBSTUB_H
#define BBSTUB_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* ── time ── */
typedef int64_t absolute_time_t;
extern absolute_time_t bbstub_now;
static inline absolute_time_t get_absolute_time(void) { return bbstub_now; }
static inline absolute_time_t make_timeout_time_ms(int ms) { return bbstub_now + (int64_t)ms * 1000; }
static inline int64_t absolute_time_diff_us(absolute_time_t a, absolute_time_t b) { return b - a; }
static inline void sleep_us(int u) { (void)u; }
static inline void sleep_ms(int m) { (void)m; }

/* ── lwIP ── */
typedef int8_t err_t;
#define ERR_OK   0
#define ERR_VAL  (-6)
#define ERR_ABRT (-13)
typedef uint16_t u16_t;
#define TCP_WRITE_FLAG_COPY 0x01
#define IPADDR_TYPE_ANY 46
#define IP_ANY_TYPE ((void *)0)
struct netif;
struct udp_pcb;
typedef struct { uint32_t addr; } ip4_addr_t;
typedef ip4_addr_t ip_addr_t;
int ip4addr_aton(const char *s, ip4_addr_t *a);
struct tcp_pcb { int id; };
struct pbuf { struct pbuf *next; void *payload; uint16_t len, tot_len; };
void pbuf_free(struct pbuf *p);
err_t tcp_write(struct tcp_pcb *pcb, const void *data, uint16_t len, uint8_t flags);
void  tcp_output(struct tcp_pcb *pcb);
err_t tcp_close(struct tcp_pcb *pcb);
void  tcp_abort(struct tcp_pcb *pcb);
void  tcp_arg(struct tcp_pcb *pcb, void *arg);
void  tcp_recv(struct tcp_pcb *pcb, err_t (*f)(void *, struct tcp_pcb *, struct pbuf *, err_t));
void  tcp_sent(struct tcp_pcb *pcb, err_t (*f)(void *, struct tcp_pcb *, u16_t));
void  tcp_err(struct tcp_pcb *pcb, void (*f)(void *, err_t));
void  tcp_recved(struct tcp_pcb *pcb, uint16_t n);
struct tcp_pcb *tcp_new_ip_type(int t);
err_t tcp_bind(struct tcp_pcb *pcb, void *ip, uint16_t port);
struct tcp_pcb *tcp_listen_with_backlog(struct tcp_pcb *pcb, int n);
void  tcp_accept(struct tcp_pcb *pcb, err_t (*f)(void *, struct tcp_pcb *, err_t));
#define mem_malloc malloc
#define mem_free   free

/* ── cyw43 / board ── */
#define CYW43_ITF_AP 1
#define CYW43_ITF_STA 0
#define CYW43_COUNTRY_WORLDWIDE 0
void cyw43_wifi_set_up(void *self, int itf, bool up, uint32_t country);
void cyw43_wifi_ap_set_ssid(void *self, size_t len, const uint8_t *buf);
void cyw43_wifi_ap_set_auth(void *self, uint32_t auth);
#define CYW43_WL_GPIO_LED_PIN 0
void cyw43_arch_gpio_put(int pin, bool on);
#define CYW43_AUTH_OPEN 0
struct netif { int dummy; };
struct { struct netif netif[2]; } cyw43_state;
void cyw43_arch_enable_ap_mode(const char *ssid, const char *pw, uint32_t auth);
void cyw43_arch_poll(void);
typedef struct { uint8_t id[8]; } pico_unique_board_id_t;
void pico_get_unique_board_id(pico_unique_board_id_t *o);
void watchdog_reboot(uint32_t a, uint32_t b, uint32_t c);
#endif
