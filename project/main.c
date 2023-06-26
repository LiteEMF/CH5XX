
#include "CH559.H"
#pragma  NOAREGS
#include  "api/api_system.h"
#include  "app/emf.h"

#include  "api/api_log.h"

void api_timer_hook(uint8_t id)
{
	if(0 == id){
		m_task_tick10us += 100;
		m_systick++;
	}
}


int main(void)
{
	api_set_sysclk(SYSCLK,HAL_SYS_FREQ);
	emf_mem_init();
	api_uart_init(UART_DEBUG_ID);	
	
	logi("****************************************\n");
	logi ("main\n");
	logi("****************************************\n");
	emf_api_init();
	emf_init();


	while(1){
		emf_handler(0);
	}
}