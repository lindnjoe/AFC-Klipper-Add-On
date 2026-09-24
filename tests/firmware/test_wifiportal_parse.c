/* Native test for the setup portal's request parsing and save state machine.
 *
 * WHY NATIVE. The two things most likely to be wrong here cannot be seen on a
 * board: a WPA2 passphrase is exactly the string that breaks a form decoder
 * (it is all punctuation, and '+' and '&' and '%' each mean something else in
 * a urlencoded body), and a POST body routinely arrives in a SECOND TCP
 * segment, so answering the first one reads credentials out of a request that
 * has not finished being sent. Both look identical on hardware to "wrong
 * password" -- the board just does not join -- which is the worst possible
 * failure to debug through an access point.
 *
 * So wifiportal.c is compiled here against stub headers and driven directly.
 * The lwIP and CYW43 calls are recording stubs; nothing pretends to be a
 * network. See tests/firmware/stubs/bbstub.h.
 */
#define BB_WIFI 1
#include "bbstub.h"

#include <stdio.h>
#include <assert.h>
#include "wificfg.h"   /* real header: the stubs below must match it exactly */
#include "vendor/dhcpserver.h"
#include "vendor/dnsserver.h"

absolute_time_t bbstub_now = 1000000;

/* ── recording stubs ── */
static char  g_written[8192];
static size_t g_wlen;
static int   g_rebooted;

err_t tcp_write(struct tcp_pcb *pcb, const void *data, uint16_t len, uint8_t f) {
    (void)pcb; (void)f;
    if (g_wlen + len < sizeof g_written) {
        memcpy(g_written + g_wlen, data, len);
        g_wlen += len;
        g_written[g_wlen] = 0;
    }
    return ERR_OK;
}
void  tcp_output(struct tcp_pcb *p) { (void)p; }
err_t tcp_close(struct tcp_pcb *p) { (void)p; return ERR_OK; }
void  tcp_abort(struct tcp_pcb *p) { (void)p; }
void  tcp_arg(struct tcp_pcb *p, void *a) { (void)p; (void)a; }
void  tcp_recv(struct tcp_pcb *p, err_t (*f)(void *, struct tcp_pcb *, struct pbuf *, err_t)) { (void)p; (void)f; }
void  tcp_sent(struct tcp_pcb *p, err_t (*f)(void *, struct tcp_pcb *, u16_t)) { (void)p; (void)f; }
void  tcp_err(struct tcp_pcb *p, void (*f)(void *, err_t)) { (void)p; (void)f; }
void  tcp_recved(struct tcp_pcb *p, uint16_t n) { (void)p; (void)n; }
struct tcp_pcb *tcp_new_ip_type(int t) { (void)t; return NULL; }
err_t tcp_bind(struct tcp_pcb *p, void *ip, uint16_t port) { (void)p; (void)ip; (void)port; return ERR_OK; }
struct tcp_pcb *tcp_listen_with_backlog(struct tcp_pcb *p, int n) { (void)p; (void)n; return NULL; }
void  tcp_accept(struct tcp_pcb *p, err_t (*f)(void *, struct tcp_pcb *, err_t)) { (void)p; (void)f; }
void  pbuf_free(struct pbuf *p) { (void)p; }
int   ip4addr_aton(const char *s, ip4_addr_t *a) { (void)s; a->addr = 0; return 1; }
void  cyw43_arch_enable_ap_mode(const char *s, const char *pw, uint32_t a) { (void)s; (void)pw; (void)a; }
void  cyw43_wifi_set_up(void *s, int i, bool u, uint32_t c) { (void)s; (void)i; (void)u; (void)c; }
void  cyw43_wifi_ap_set_ssid(void *s, size_t l, const uint8_t *b) { (void)s; (void)l; (void)b; }
void  cyw43_wifi_ap_set_auth(void *s, uint32_t a) { (void)s; (void)a; }
void  cyw43_arch_poll(void) {}
void  cyw43_arch_gpio_put(int pin, bool on) { (void)pin; (void)on; }
static unsigned g_beats;
void  wifilink_beat(void) { g_beats++; }
static unsigned g_stage;
void  wifilink_stage(uint32_t s) { g_stage = s; }
static const uint8_t g_id[8] = {0,1,2,3,4,5,0xAB,0xCD};
const uint8_t *wifilink_board_id(void) { return g_id; }
void  pico_get_unique_board_id(pico_unique_board_id_t *o) { memset(o, 0xAB, sizeof *o); }
void  watchdog_reboot(uint32_t a, uint32_t b, uint32_t c) { (void)a; (void)b; (void)c; g_rebooted = 1; }
/* The vendored DHCP and DNS servers are real code with real lwIP guts; this
 * test is about the HTTP side, so they are stubbed rather than compiled. */
void dhcp_server_init(dhcp_server_t *d, struct netif *n, ip_addr_t *i, ip_addr_t *m) { (void)d; (void)n; (void)i; (void)m; }
void dhcp_server_deinit(dhcp_server_t *d) { (void)d; }
void dns_server_init(dns_server_t *d, struct netif *n, ip_addr_t *i) { (void)d; (void)n; (void)i; }
void dns_server_deinit(dns_server_t *d) { (void)d; }

/* ── wificfg stub: record what the portal hands over ── */
static char g_ssid[64], g_pass[128], g_key[128];
static int  g_staged, g_result = -2;
bool wificfg_load(wificfg_t *o) { (void)o; return false; }
int  wificfg_save(const wificfg_t *i) { (void)i; return 1; }
int  wificfg_wipe(void) { return 1; }
bool wificfg_stage(const char *ssid, const char *pass, const char *key) {
    snprintf(g_ssid, sizeof g_ssid, "%s", ssid ? ssid : "");
    snprintf(g_pass, sizeof g_pass, "%s", pass ? pass : "");
    snprintf(g_key,  sizeof g_key,  "%s", key  ? key  : "");
    g_staged++;
    return true;
}
int wificfg_service(void) { return -2; }
int wificfg_last_result(void) { return g_result; }

#include "wifiportal.c"

/* ── helpers ── */
static conn_t *g_conn;
static struct tcp_pcb g_pcb = { 1 };

static void reset(void) {
    g_wlen = 0; g_written[0] = 0; g_rebooted = 0;
    g_staged = 0; g_result = -2; g_ssid[0] = g_pass[0] = g_key[0] = 0;
    s_save = SAVE_IDLE; s_save_pcb = NULL;
    free(g_conn);
    g_conn = calloc(1, sizeof(conn_t));
}

/* Push one TCP segment in, exactly as on_recv would receive it. */
static void feed(const char *seg) {
    struct pbuf p = { 0 };
    p.payload = (void *)seg;
    p.len = (uint16_t)strlen(seg);
    p.tot_len = p.len;
    on_recv(g_conn, &g_pcb, &p, ERR_OK);
}

static int fails;
static void check(int cond, const char *what) {
    if (!cond) { printf("FAIL: %s\n", what); fails++; }
}

int main(void) {
    /* 1. urldecode: the three characters that are not themselves. */
    char out[64];
    urldecode("a+b", 3, out, sizeof out);
    check(strcmp(out, "a b") == 0, "'+' decodes to space");
    urldecode("%21%40%23", 9, out, sizeof out);
    check(strcmp(out, "!@#") == 0, "%XX decodes to bytes");
    urldecode("%2B", 3, out, sizeof out);
    check(strcmp(out, "+") == 0, "%2B is a literal plus, not a space");
    urldecode("100%", 4, out, sizeof out);
    check(strcmp(out, "100%") == 0, "a trailing bare %% is left alone");

    /* 2. form_field must not find "pass" inside another field's name. */
    char v[64];
    check(form_field("bypass=no&pass=yes", "pass", v, sizeof v)
          && strcmp(v, "yes") == 0, "field name matches whole, not substring");
    check(!form_field("ssidx=a", "ssid", v, sizeof v),
          "a longer field name is not a match");

    /* 3. A GET is complete at the blank line; a POST is not complete until
     *    Content-Length bytes have followed it. */
    reset();
    strcpy(g_conn->buf, "GET / HTTP/1.1\r\nHost: x\r\n\r\n");
    g_conn->len = (uint16_t)strlen(g_conn->buf);
    check(request_complete(g_conn), "GET complete at blank line");

    reset();
    strcpy(g_conn->buf, "POST /save HTTP/1.1\r\nContent-Length: 20\r\n\r\n");
    g_conn->len = (uint16_t)strlen(g_conn->buf);
    check(!request_complete(g_conn), "POST with no body yet is incomplete");

    /* 4. THE ONE THAT MATTERS: a form POST split across two segments, with a
     *    passphrase made of the characters that mean something in a body. */
    reset();
    const char *body = "ssid=My+Home+2.4G&pass=S3cret%2BP%40ss%26word%21";
    char head[256];
    snprintf(head, sizeof head,
             "POST /save HTTP/1.1\r\nHost: 192.168.4.1\r\n"
             "Content-Type: application/x-www-form-urlencoded\r\n"
             "Content-Length: %u\r\n\r\n", (unsigned)strlen(body));
    /* first segment: the head and the first few body bytes */
    char seg1[320];
    snprintf(seg1, sizeof seg1, "%s%.6s", head, body);
    feed(seg1);
    check(g_staged == 0, "nothing staged from a partial POST");
    check(g_wlen == 0, "nothing answered from a partial POST");

    feed(body + 6);                     /* the rest arrives */
    check(g_staged == 1, "staged once the whole body arrived");
    check(strcmp(g_ssid, "My Home 2.4G") == 0, "ssid decoded across segments");
    check(strcmp(g_pass, "S3cret+P@ss&word!") == 0,
          "passphrase with + @ & ! survives decoding");
    check(s_save == SAVE_WAITING, "waiting on core0, not answered yet");
    check(g_wlen == 0, "no page sent before the write is confirmed");

    /* core0 says it wrote: the loop -- not the callback -- sends the page. */
    g_result = 1;
    save_pump();
    check(strstr(g_written, "200 OK") != NULL, "saved page is a 200");
    check(strstr(g_written, "My Home 2.4G") != NULL, "saved page names the ssid");
    check(g_key[0] == '\0',
          "a form with no key field leaves the link key empty (open link)");
    check(strstr(g_written, "S3cret") == NULL,
          "the passphrase is never echoed back to the browser");
    check(s_save == SAVE_FLUSHING, "flushing, reboot deferred");
    check(!g_rebooted, "does not reboot before the page is flushed");

    bbstub_now += 2000 * 1000;          /* let the flush window expire */
    save_pump();
    check(g_rebooted, "reboots once the page has had its flush window");

    /* 5. core0 failing to answer must not claim a save. */
    reset();
    feed(seg1); feed(body + 6);
    check(s_save == SAVE_WAITING, "waiting again");
    bbstub_now += 5000 * 1000;          /* past the 3 s deadline, result -2 */
    save_pump();
    check(!g_rebooted, "no reboot when the write was never confirmed");
    check(strstr(g_written, "did not confirm") != NULL,
          "says the write was not confirmed rather than claiming success");

    /* 6. Anything else redirects, so a captive-portal probe opens the form. */
    reset();
    feed("GET /generate_204 HTTP/1.1\r\nHost: x\r\n\r\n");
    check(strstr(g_written, "302") != NULL, "unknown path redirects");
    check(strstr(g_written, "Location: http://192.168.4.1/") != NULL,
          "redirect points at the portal");

    /* 5. THE LINK KEY comes through the same decoder as the passphrase, and
     *    is never echoed to the browser -- it is the one secret that would let
     *    anything on the network drive the bridge. */
    reset();
    {
        const char *b5 = "ssid=Net&pass=pw&key=k%40y+w%26th+junk%21";
        char req5[512];
        snprintf(req5, sizeof req5,
                 "POST /save HTTP/1.1\r\nContent-Length: %u\r\n\r\n%s",
                 (unsigned)strlen(b5), b5);
        feed(req5);
        check(g_staged == 1, "key form staged once");
        check(strcmp(g_key, "k@y w&th junk!") == 0,
              "link key survives the same decoding as the passphrase");
        check(strstr(g_written, "k@y w&th junk!") == NULL,
              "the link key is never echoed back to the browser");
    }

    if (fails == 0) printf("all wifiportal parse tests passed\n");
    return fails ? 1 : 0;
}
