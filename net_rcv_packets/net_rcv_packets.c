/*
内核接收网络数据包源码分析  
*/

/*
网卡硬中断注册的处理函数
*/
static irqreturn_t igb_msix_ring(int irq, void *data)
{
	struct igb_q_vector *q_vector = data;

	/* Write the ITR value calculated from the previous interrupt. */
	//硬件中断频率
	igb_write_itr(q_vector);
	//NAPI处理队列中的数据包(软中断)
	napi_schedule(&q_vector->napi);
	//成功处理软中断 扩充IRQ_NONE：未处理软中断；IRQ_WAKE_THREAD：延迟执行
	return IRQ_HANDLED;
}

static inline void napi_schedule(struct napi_struct *n)
{
	if (napi_schedule_prep(n))
		__napi_schedule(n);
}

void __napi_schedule(struct napi_struct *n)
{
	unsigned long flags;

	local_irq_save(flags);
	____napi_schedule(this_cpu_ptr(&softnet_data), n);
	local_irq_restore(flags);
}

/* Called with irq disabled */
/*
 * ____napi_schedule - 内部函数，调度 NAPI 以处理网络数据包
 * @sd: 指向当前 CPU 的 softnet_data 结构体的指针
 * @napi: 指向要调度的 NAPI 结构体的指针
 * 中断被禁用情况下使用
*/
static inline void ____napi_schedule(struct softnet_data *sd,
				     struct napi_struct *napi)
{
	struct task_struct *thread;
	//禁用中断
	lockdep_assert_irqs_disabled();
	//NAPI处于线程化模式
	if (test_bit(NAPI_STATE_THREADED, &napi->state)) {
		/* Paired with smp_mb__before_atomic() in
		 * napi_enable()/dev_set_threaded().
		 * Use READ_ONCE() to guarantee a complete
		 * read on napi->thread. Only call
		 * wake_up_process() when it's not NULL.
		 */
		thread = READ_ONCE(napi->thread);
		if (thread) {
			/* Avoid doing set_bit() if the thread is in
			 * INTERRUPTIBLE state, cause napi_thread_wait()
			 * makes sure to proceed with napi polling
			 * if the thread is explicitly woken from here.
			 */
			/*
			1、TASK_INTERRUPTIBLE 线程处于可中断的睡眠状态
			2、当线程处于运行态时，NAPI_STATE_SCHED_THREADED NAPI机制轮询
			*/
			if (READ_ONCE(thread->__state) != TASK_INTERRUPTIBLE)
				set_bit(NAPI_STATE_SCHED_THREADED, &napi->state);
				//唤醒对应NAPI线程处理网络数据包
			wake_up_process(thread);
			return;
		}
	}
	//线程处于非线程化模式：将驱动napi_struct传递的poll_list添加至softnet_data的poll_list
	list_add_tail(&napi->poll_list, &sd->poll_list);
	//触发软中断NET_RX_SOFTIR
	__raise_softirq_irqoff(NET_RX_SOFTIRQ);
}

/*
功能：中断处理函数中触发softirq
@nr: 触发的软中断号
*/
void __raise_softirq_irqoff(unsigned int nr)
{
	//禁用中断
	lockdep_assert_irqs_disabled();
	//软中断触发
	trace_softirq_raise(nr);
	/*
	1、标记指定的软中断类型为挂起状态
	2、将第 nr 位对应的软中断标志位与 1 做或运算，将该位设置为1。
	例如：
		软中断编号：3
		初始值：0b0000 并未挂起
		1UL << nr：0b1000
		或运算：0b0000 | 0b1000
		位图结果：0b1000 
		结论：编号3的软中断挂起
	*/
	or_softirq_pending(1UL << nr);
}

/*
功能：软中断：特定cpu上 ksoftirqd 内核线程是否应该运行
*/
static int ksoftirqd_should_run(unsigned int cpu)
{
	return local_softirq_pending();
}
/*
区别：
	1、硬中断和软中断均调用local_softirq_pending 
	2、硬中断设置NAT_RX_SOFTIRQ,软中断读取
功能：
	实际内核线程处理函数，当系统中存在挂起的软中断，触发处理
@cpu：特定cpu上
*/
static void run_ksoftirqd(unsigned int cpu)
{
	//开始执行softirqd
	ksoftirqd_run_begin();
	//当前cpu上是否存在挂起的软中断
	if (local_softirq_pending()) {
		/*
		 * We can safely run softirq on inline stack, as we are not deep
		 * in the task stack here.
		 */
		//实际软中断处理函数
		__do_softirq();
		//结束执行softirqd
		ksoftirqd_run_end();
		//检查是否需要让出cpu去调度其他任务
		cond_resched();
		return;
	}
	//无软中断挂起，继续执行
	ksoftirqd_run_end();
}

/*
功能：处理软中断核心
硬中断在哪个CPU上被响应其软中断也在此CPU上处理

*/
asmlinkage __visible void __softirq_entry __do_softirq(void)
{
	/*
	1、end：处理软中断最大时间，单位jiffies
	2、MAX_SOFTIRQ_TIME：预定义的处理软中断的最大时间，防止软中断占用过多的 CPU 时间
	3、MAX_SOFTIRQ_RESTART：软中断处理最大重启次数
	*/
	unsigned long end = jiffies + MAX_SOFTIRQ_TIME;
	unsigned long old_flags = current->flags;
	int max_restart = MAX_SOFTIRQ_RESTART;
	struct softirq_action *h;
	bool in_hardirq;
	__u32 pending;
	int softirq_bit;

	/*
	 * Mask out PF_MEMALLOC as the current task context is borrowed for the
	 * softirq. A softirq handled, such as network RX, might set PF_MEMALLOC
	 * again if the socket is related to swapping.
	 */
	//清除PF_MEMALLOC 标志，避免软中断期间错误分配内存
	current->flags &= ~PF_MEMALLOC;
	//获取当前cpu挂起的软中断位图
	pending = local_softirq_pending();
	//标记软中断处理开始
	softirq_handle_begin();
	//记录当前是否在硬中断上下文中
	in_hardirq = lockdep_softirq_start();
	//更新当前任务进入软中断处理的统计信息
	account_softirq_enter(current);

restart:
	/* Reset the pending bitmask before enabling irqs */
	set_softirq_pending(0);
	//开中断
	local_irq_enable();
	//软中断处理向量起始位置
	h = softirq_vec;
	//遍历挂起的软中断
	while ((softirq_bit = ffs(pending))) {
		unsigned int vec_nr;
		int prev_count;
		/*
		softirq_vec 是一个指向 softirq_action 数组第一个元素的指针，数组的每个元素对应一个特定类型的软中断处理程序。
		h += softirq_bit - 1; 将指针 h 从 softirq_vec 起始位置移动到该位置处的 softirq_action 结构体。
		*/
		//将指针移动到下一个要处理的软中断
		h += softirq_bit - 1;
		/*
		h - softirq_vec 计算出指针 h 与 softirq_vec 数组起始位置之间的偏移量，即软中断的向量号 vec_nr	
		*/
		//计算softirq向量号 
		vec_nr = h - softirq_vec;
		//保存当前的抢占计数
		prev_count = preempt_count();
		//增加当前cpu软中断计数
		kstat_incr_softirqs_this_cpu(vec_nr);
		//记录软中断入口跟踪信息
		trace_softirq_entry(vec_nr);
		//执行softirq处理程序
		h->action(h);
		//记录软中断退出的跟踪信息
		trace_softirq_exit(vec_nr);
		//检查软中断处理前后抢占计数是否一致
		if (unlikely(prev_count != preempt_count())) {
			pr_err("huh, entered softirq %u %s %p with preempt_count %08x, exited with %08x?\n",
			       vec_nr, softirq_to_name[vec_nr], h->action,
			       prev_count, preempt_count());
				   //不一致恢复到原来的抢占计数
			preempt_count_set(prev_count);
		}
		//移动到下一个softirq
		h++;
		//清除softirq位
		pending >>= softirq_bit;
	}
    // 如果当前任务是 ksoftirqd，标记 RCU 快速检查点
	if (!IS_ENABLED(CONFIG_PREEMPT_RT) &&
	    __this_cpu_read(ksoftirqd) == current)
		rcu_softirq_qs();
	//关中断
	local_irq_disable();
	//再次获取当前 CPU 的软中断位图
	pending = local_softirq_pending();
	//存在挂起的软中断
	if (pending) {
		// 检查是否超时、需要调度或已达最大重启次数
		if (time_before(jiffies, end) && !need_resched() &&
		    --max_restart)
			goto restart;
  		// 如果无法继续处理，唤醒 ksoftirqd 处理剩余的软中断
		wakeup_softirqd();
	}
 	// 更新当前任务退出软中断处理的统计信息
	account_softirq_exit(current);
	// 标记软中断处理的结束
	lockdep_softirq_end(in_hardirq);
    // 标记软中断处理结束，通常用于跟踪和调试
    softirq_handle_end();
    // 恢复任务的原始标志状态
    current_restore_flags(old_flags, PF_MEMALLOC);
}

/*
功能：
	1、获取softnet_data
	2、poll_list遍历
	3、执行网卡驱动注册到的poll函数igb_poll
*/
static __latent_entropy void net_rx_action(struct softirq_action *h)
{
	//获取当前cpu的softnet_data数据结构
	struct softnet_data *sd = this_cpu_ptr(&softnet_data);
	/*计算软中断执行时间
	jiffies 记录系统启动以来经过的时间
	usecs_to_jiffies 微妙转换成jiffies
	*/
	unsigned long time_limit = jiffies +
		usecs_to_jiffies(READ_ONCE(netdev_budget_usecs));
	//允许处理的最大数据包数量
	int budget = READ_ONCE(netdev_budget);
	//初始化空链表list、repoll存放待处理的 napi_struct
	LIST_HEAD(list);
	LIST_HEAD(repoll);
	//关闭当前CPU硬中断，防止poll_list重复添加
	local_irq_disable();
	//将sd->poll_list中的所有元素移动到list链表中
	list_splice_init(&sd->poll_list, &list);
	local_irq_enable();

	for (;;) {
		struct napi_struct *n;
		 // 释放被延迟的 sk_buff 缓冲区，以防内存泄漏
		skb_defer_free_flush(sd);
		// 检查 list 链表是否为空，以及是否有等待处理的 RPS IPI 或 repoll 链表中的 napi_struct
		if (list_empty(&list)) {
			if (!sd_has_rps_ipi_waiting(sd) && list_empty(&repoll))
				goto end;
			break;
		}
		//获取list链表第一个napi_struct 元素地址
		n = list_first_entry(&list, struct napi_struct, poll_list);
       //处理 napi_struct 对应的网络数据包，从预算中扣除处理的数据包数量
		budget -= napi_poll(n, &repoll);

		/* If softirq window is exhausted then punt.
		 * Allow this to run for 2 jiffies since which will allow
		 * an average latency of 1.5/HZ.
		 */
		//是否耗尽软中断处理的预算/处理时间超过允许的最大值
		if (unlikely(budget <= 0 ||
			     time_after_eq(jiffies, time_limit))) {
			sd->time_squeeze++;
			break;
		}
	}

	local_irq_disable();

	list_splice_tail_init(&sd->poll_list, &list);
	list_splice_tail(&repoll, &list);
	list_splice(&list, &sd->poll_list);
	if (!list_empty(&sd->poll_list))
		__raise_softirq_irqoff(NET_RX_SOFTIRQ);// 触发NET_RX_SOFTIRQ软中断

	net_rps_action_and_irq_enable(sd);// 处理RPS（接收包调度）并启用本地中断
end:;
}

//igb网卡驱动的NAPI 轮询函数，处理网络驱动中的接收和发送数据包
/**
 *  igb_poll - NAPI Rx polling callback
 *  @napi: napi polling structure
 *  @budget: count of how many packets we should handle
 **/
static int igb_poll(struct napi_struct *napi, int budget)
{
	//获取napi结构体的igb_q_vector结构体
	struct igb_q_vector *q_vector = container_of(napi,
						     struct igb_q_vector,
						     napi);
	bool clean_complete = true;
	int work_done = 0;//已处理的数据包数量

#ifdef CONFIG_IGB_DCA
	// 如果启用了直接缓存访问 (DCA)，则更新其设置,DCA 可以提高网络性能，通过将接收到的数据直接放入目标 CPU 的缓存中
	if (q_vector->adapter->flags & IGB_FLAG_DCA_ENABLED)
		igb_update_dca(q_vector);
#endif
    // 存在传输 (TX) 队列，清理传输中断
	if (q_vector->tx.ring)
		clean_complete = igb_clean_tx_irq(q_vector, budget);
    // 存在接收 (RX) 队列，清理接收中断并处理接收到的数据包
	if (q_vector->rx.ring) {
		//清理接收中断并获取清理数据包的数量
		int cleaned = igb_clean_rx_irq(q_vector, budget);
		//更新已处理的包数量
		work_done += cleaned;
		//清理的数据包达到预算值，存在未完成的工作
		if (cleaned >= budget)
			clean_complete = false;
	}
	//所有工作完成，返回预算值继续轮询
	/* If all work not completed, return budget and keep polling */
	if (!clean_complete)
		return budget;

	/* Exit the polling mode, but don't re-enable interrupts if stack might
	 * poll us due to busy-polling
	 */
	//检查当前NAPI实例已经完成了所有待处理工作，退出轮询
	if (likely(napi_complete_done(napi, work_done)))
	//开中断
		igb_ring_irq_enable(q_vector);

	return work_done;
}

/*
功能：
	处理网络适配器的接收中断,清理接收到的网络数据包
@q_vector:队列，接收数据包处理相关的各种资源和状态信息，包括接收队列（RX ring）、适配器信息、网络栈信息等
@budget:最多可以处理的数据包数量
*/
static int igb_clean_rx_irq(struct igb_q_vector *q_vector, const int budget)
{
	//网络适配器
	struct igb_adapter *adapter = q_vector->adapter;
	//rx 队列
	struct igb_ring *rx_ring = q_vector->rx.ring;
	//socket 缓冲区(sk_buff)
	struct sk_buff *skb = rx_ring->skb;
	unsigned int total_bytes = 0, total_packets = 0;
	u16 cleaned_count = igb_desc_unused(rx_ring);
	unsigned int xdp_xmit = 0;
	struct xdp_buff xdp;
	u32 frame_sz = 0;
	int rx_buf_pgcnt;

	/* Frame size depend on rx_ring setup when PAGE_SIZE=4K */
//内存页面大小<8192,计算帧大小
#if (PAGE_SIZE < 8192)
	frame_sz = igb_rx_frame_truesize(rx_ring, 0);
#endif
	xdp_init_buff(&xdp, frame_sz, &rx_ring->xdp_rxq);
	//遍历预算内 rx 队列数据包
	while (likely(total_packets < budget)) {
		union e1000_adv_rx_desc *rx_desc;
		struct igb_rx_buffer *rx_buffer;
		ktime_t timestamp = 0;
		int pkt_offset = 0;
		unsigned int size;
		void *pktbuf;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= IGB_RX_BUFFER_WRITE) {
			igb_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		rx_desc = IGB_RX_DESC(rx_ring, rx_ring->next_to_clean);
		//数据包大小
		size = le16_to_cpu(rx_desc->wb.upper.length);
		if (!size)
			break;

		/* This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we know the
		 * descriptor has been written back
		 */
		// 内存屏障，确保在读取描述符的其他字段之前，已经读取了完整的大小信息
		dma_rmb();
		/*
		rx_buffer->page：指向 struct page 的指针，struct page 结构体表示内核中的一个物理页面
		page_address(rx_buffer->page)：页面起始地址
		rx_buffer->page_offset：数据包在页面中的偏移量
		pktbuf：数据包在实际内存起始位置
		例如：
			页面大小：4KB
			page_address(rx_buffer->page)：地址0x10000000
			rx_buffer->page_offset：128字节
			pktbuf：0x10000000 + 128 = 0x10000080

		*/
		rx_buffer = igb_get_rx_buffer(rx_ring, size, &rx_buf_pgcnt);
		pktbuf = page_address(rx_buffer->page) + rx_buffer->page_offset;

		/* pull rx packet timestamp if available and valid */
		/*
		数据包中包含时间戳，提取
		rx_ring->q_vector：队列向量
		&timestamp：存储时间戳
		pkt_offset += ts_hdr_len：数据包偏移量 + 时间戳长度 跳过时间戳部分
		size -= ts_hdr_len：减少数据包大小减少时间戳长度 处理部分不包括时间戳
		*/
		if (igb_test_staterr(rx_desc, E1000_RXDADV_STAT_TSIP)) {
			int ts_hdr_len;
			ts_hdr_len = igb_ptp_rx_pktstamp(rx_ring->q_vector,
							 pktbuf, &timestamp);

			pkt_offset += ts_hdr_len;
			size -= ts_hdr_len;
			}
	
		 //如果当前没有 skb，则准备 XDP 缓冲区
		/* retrieve a buffer from the ring */
		if (!skb) {
			// 计算硬件起始地址，将pktbuf（数据包起始地址）向前移动 igb_rx_offset(rx_ring) 个字节
			unsigned char *hard_start = pktbuf - igb_rx_offset(rx_ring);
			// 计算数据包的实际偏移量，加上偏移值和 RX ring 的偏移值
			unsigned int offset = pkt_offset + igb_rx_offset(rx_ring);

			xdp_prepare_buff(&xdp, hard_start, offset, size, true);
			xdp_buff_clear_frags_flag(&xdp);
//PAGE_SIZE 大于 4K，根据数据包大小调整帧大小
#if (PAGE_SIZE > 4096)
			/* At larger PAGE_SIZE, frame_sz depend on len size */
			xdp.frame_sz = igb_rx_frame_truesize(rx_ring, size);
#endif
		// 执行 XDP 程序
			skb = igb_run_xdp(adapter, rx_ring, &xdp);
		}

		if (IS_ERR(skb)) {
			unsigned int xdp_res = -PTR_ERR(skb);
			// 如果 XDP 程序返回了传输或重定向的结果，处理相应的缓冲区
			if (xdp_res & (IGB_XDP_TX | IGB_XDP_REDIR)) {
				xdp_xmit |= xdp_res;
				igb_rx_buffer_flip(rx_ring, rx_buffer, size);
			} else {
				rx_buffer->pagecnt_bias++;
			}
			total_packets++;
			total_bytes += size;
		} else if (skb)
		     // 如果 `skb` 有效，添加到 `skb` 中
			igb_add_rx_frag(rx_ring, rx_buffer, skb, size);
		else if (ring_uses_build_skb(rx_ring))
		 // 构建一个新的 `skb`
			skb = igb_build_skb(rx_ring, rx_buffer, &xdp,
					    timestamp);
		else
		  // 构建标准的 `skb`  
			skb = igb_construct_skb(rx_ring, rx_buffer,
						&xdp, timestamp);

		/* exit if we failed to retrieve a buffer */
		if (!skb) {
			rx_ring->rx_stats.alloc_failed++;
			rx_buffer->pagecnt_bias++;
			break;
		}

		igb_put_rx_buffer(rx_ring, rx_buffer, rx_buf_pgcnt);
		cleaned_count++;

		/* fetch next buffer in frame if non-eop */
		if (igb_is_non_eop(rx_ring, rx_desc))
			continue;

		/* verify the packet layout is correct */
		if (igb_cleanup_headers(rx_ring, rx_desc, skb)) {
			skb = NULL;
			continue;
		}

		/* probably a little skewed due to removing CRC */
		//更新总字节数
		total_bytes += skb->len;
		// 填充校验和、时间戳、VLAN 和协议字段
		/* populate checksum, timestamp, VLAN, and protocol */
		igb_process_skb_fields(rx_ring, rx_desc, skb);
 		// 传递给上层网络栈进行进一步处理
		napi_gro_receive(&q_vector->napi, skb);

		/* reset skb pointer */
		skb = NULL;

		/* update budget accounting */
		total_packets++;
	}

	/* place incomplete frames back on ring for completion */
	rx_ring->skb = skb;

	if (xdp_xmit & IGB_XDP_REDIR)
		xdp_do_flush();

	if (xdp_xmit & IGB_XDP_TX) {
		struct igb_ring *tx_ring = igb_xdp_tx_queue_mapping(adapter);

		igb_xdp_ring_update_tail(tx_ring);
	}
	//更新
	u64_stats_update_begin(&rx_ring->rx_syncp);
	rx_ring->rx_stats.packets += total_packets;
	rx_ring->rx_stats.bytes += total_bytes;
	u64_stats_update_end(&rx_ring->rx_syncp);
	q_vector->rx.total_packets += total_packets;
	q_vector->rx.total_bytes += total_bytes;

	if (cleaned_count)
		igb_alloc_rx_buffers(rx_ring, cleaned_count);
   // 返回已处理的数据包数量
	return total_packets;
}

/*
功能：
	1、处理网络数据包接收
	2、gro_result_t:网卡GRO特性合并小数据包，减少传递到网络栈数据包数量
*/
gro_result_t napi_gro_receive(struct napi_struct *napi, struct sk_buff *skb)
{
	gro_result_t ret;

	skb_mark_napi_id(skb, napi);
	trace_napi_gro_receive_entry(skb);
	//重置GRO偏移量
	skb_gro_reset_offset(skb, 0);
	//dev_gro_receive：GRO操作
	ret = napi_skb_finish(napi, skb, dev_gro_receive(napi, skb));
	trace_napi_gro_receive_exit(ret);

	return ret;
}

/*
 功能：
 	 根据 GRO 结果处理接收到的数据包
*/
static gro_result_t napi_skb_finish(struct napi_struct *napi,
				    struct sk_buff *skb,
				    gro_result_t ret)
{
	switch (ret) {
	//数据包未被合并，gro_normal_one处理单个数据包
	case GRO_NORMAL:
		gro_normal_one(napi, skb, 1);
		break;
	//数据包被合并且应该被释放
	case GRO_MERGED_FREE:
		if (NAPI_GRO_CB(skb)->free == NAPI_GRO_FREE_STOLEN_HEAD)
		//free header
			napi_skb_free_stolen_head(skb);
		// 如果 `skb` 是一个可用的克隆缓冲区，直接释放 `skb`
		else if (skb->fclone != SKB_FCLONE_UNAVAILABLE)
			__kfree_skb(skb);
		else
			__kfree_skb_defer(skb);
		break;
	//数据包已经被处理
	case GRO_HELD:
	case GRO_MERGED:
	case GRO_CONSUMED:
		break;
	}

	return ret;
}


static inline void gro_normal_one(struct napi_struct *napi, struct sk_buff *skb, int segs)
{
	//将skb添加到NAPI接收rx_list的尾部
	list_add_tail(&skb->list, &napi->rx_list);
	//更新NAPI接收计数,数据包片段数
	napi->rx_count += segs;
   // 如果接收计数器达到或超过批处理阈值 (gro_normal_batch)
	if (napi->rx_count >= READ_ONCE(gro_normal_batch))
	//批量处理接收的数据包列表
		gro_normal_list(napi);
}

/*
功能：
1、批量处理napi结构体的接收数据包列表
2、清除 & 重置

*/
/* Pass the currently batched GRO_NORMAL SKBs up to the stack. */
static inline void gro_normal_list(struct napi_struct *napi)
{
	//NAPI的接收计数器为0
	if (!napi->rx_count)
		return;
	//处理数据包
	netif_receive_skb_list_internal(&napi->rx_list);
	//重置NAPI接收列表
	INIT_LIST_HEAD(&napi->rx_list);
	//重置计数器
	napi->rx_count = 0;
}

/*
功能：
	1、处理sk_buff列表
	2、传递交付上层网络协议栈

*/
void netif_receive_skb_list_internal(struct list_head *head)
{
	struct sk_buff *skb, *next;
	struct list_head sublist;
	//初始化一个子列表sublist，暂时存放经过初步处理的数据包
	INIT_LIST_HEAD(&sublist);
	//遍历skb列表
	list_for_each_entry_safe(skb, next, head, list) {
		net_timestamp_check(READ_ONCE(netdev_tstamp_prequeue), skb);
		//原始列表删除skb，初始化列表头
		skb_list_del_init(skb);
		//不需要延迟处理时间戳，skb添加至子列表
		if (!skb_defer_rx_timestamp(skb))
			list_add_tail(&skb->list, &sublist);
	}
	//子列表合并回原始列表，初始化子列表
	list_splice_init(&sublist, head);

	rcu_read_lock();
#ifdef CONFIG_RPS
/*
注释：
	RPS策略：
		1、优化网络数据包处理机制，数据包分配到不同的CPU核心
		2、提高多核系统并行处理能力，减少单个CPU核心负载，提高吞吐量

*/
	//启用RPS，根据RPS策略将数据包分配到相应CPU
	if (static_branch_unlikely(&rps_needed)) {
		//遍历skb进行RPS策略处理
		list_for_each_entry_safe(skb, next, head, list) {
			struct rps_dev_flow voidflow, *rflow = &voidflow;
			//计算当前skb的目标CPU核心
			int cpu = get_rps_cpu(skb->dev, skb, &rflow);
			//cpu>=0 数据包被成功分配到一个CPU
			if (cpu >= 0) {
				/* Will be handled, remove from list */
				skb_list_del_init(skb);
				//数据包添加指定CPU处理队列
				enqueue_to_backlog(skb, cpu, &rflow->last_qtail);
			}
		}
	}
#endif
	__netif_receive_skb_list(head);
	rcu_read_unlock();
}

/*
功能：
	1、遍历skb列表，对每个skb处理
	2、根据数据包类型和原始设备将数据包分成不同列表

*/
static void __netif_receive_skb_list_core(struct list_head *head, bool pfmemalloc)
{
	/* Fast-path assumptions:
	 * - There is no RX handler.
	 * - Only one packet_type matches.
	 * If either of these fails, we will end up doing some per-packet
	 * processing in-line, then handling the 'last ptype' for the whole
	 * sublist.  This can't cause out-of-order delivery to any single ptype,
	 * because the 'last ptype' must be constant across the sublist, and all
	 * other ptypes are handled per-packet.
	 */
	/* Current (common) ptype of sublist */
	struct packet_type *pt_curr = NULL;
	/* Current (common) orig_dev of sublist */
	struct net_device *od_curr = NULL;
	struct list_head sublist;
	struct sk_buff *skb, *next;

	INIT_LIST_HEAD(&sublist);
	list_for_each_entry_safe(skb, next, head, list) {
		struct net_device *orig_dev = skb->dev;
		struct packet_type *pt_prev = NULL;

		skb_list_del_init(skb);
		//获取数据包类型packet_type
		__netif_receive_skb_core(&skb, pfmemalloc, &pt_prev);
		if (!pt_prev)
			continue;
		// 如果当前子列表的 packet_type 或 orig_dev 发生变化
		if (pt_curr != pt_prev || od_curr != orig_dev) {
			/* dispatch old sublist */
			__netif_receive_skb_list_ptype(&sublist, pt_curr, od_curr);
			/* start new sublist */
			INIT_LIST_HEAD(&sublist);
			pt_curr = pt_prev;
			od_curr = orig_dev;
		}
		//将当前skb添加到list尾部
		list_add_tail(&skb->list, &sublist);
	}

	/* dispatch final sublist */
	__netif_receive_skb_list_ptype(&sublist, pt_curr, od_curr);
}

//传递数据包
static inline int deliver_skb(struct sk_buff *skb,
			      struct packet_type *pt_prev,
			      struct net_device *orig_dev)
{
	//检查数据包片段，确保片段引用计数为0
	if (unlikely(skb_orphan_frags_rx(skb, GFP_ATOMIC)))
		return -ENOMEM;
	//增加sk_buff引用次数
	refcount_inc(&skb->users);
	// 调用协议层注册的处理函数，IP包进入ip_rcv
	return pt_prev->func(skb, skb->dev, pt_prev, orig_dev);
}
