
/**
 *  __igb_open - Called when a network interface is made active
 *  @netdev: network interface device structure
 *  @resuming: indicates whether we are in a resume call
 *
 *  Returns 0 on success, negative value on failure
 *
 *  The open entry point is called when a network interface is made
 *  active by the system (IFF_UP).  At this point all resources needed
 *  for transmit and receive operations are allocated, the interrupt
 *  handler is registered with the OS, the watchdog timer is started,
 *  and the stack is notified that the interface is ready.
 **/
static int __igb_open(struct net_device *netdev, bool resuming)
{
	struct igb_adapter *adapter = netdev_priv(netdev);
	//网卡硬件信息
	struct e1000_hw *hw = &adapter->hw;
	//网卡关联的PCI设备信息
	struct pci_dev *pdev = adapter->pdev;
	int err;
	int i;

	/* disallow open during test */
	//设备测试
	if (test_bit(__IGB_TESTING, &adapter->state)) {
		WARN_ON(resuming);
		return -EBUSY;
	}
	//获取设备电源管理同步状态
	if (!resuming)
		pm_runtime_get_sync(&pdev->dev);
	//当网络设备没有连接的时候需要关闭网络载波
	netif_carrier_off(netdev);
	//分配传输描述符数组，为传输队列分配资源
	/* allocate transmit descriptors */
	err = igb_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;
	//分配接收描述符数组并分配了RingBuffer、建立了内存和Rx队列映射关系
	/* allocate receive descriptors */
	err = igb_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;
	//启动网卡链接，准备数据传输
	igb_power_up_link(adapter);

	/* before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.
	 */
	//配置网卡适配器
	igb_configure(adapter);
	//请求分配中断资源
	err = igb_request_irq(adapter);
	if (err)
		goto err_req_irq;
	//设置网络设备实际的传输队列数量
	/* Notify the stack of the actual queue counts. */
	err = netif_set_real_num_tx_queues(adapter->netdev,
					   adapter->num_tx_queues);
	if (err)
		goto err_set_queues;
	//设置网络设备实际的接收队列数量
	err = netif_set_real_num_rx_queues(adapter->netdev,
					   adapter->num_rx_queues);
	if (err)
		goto err_set_queues;

	/* From here on the code is the same as igb_up() */
	//清除适配器的__IGB_DOWN标志，适配器处于启动状态
	clear_bit(__IGB_DOWN, &adapter->state);
	//启用NAPI轮询机制，循环启用适配器所有队列向量的NAPI结构，
	for (i = 0; i < adapter->num_q_vectors; i++)
		napi_enable(&(adapter->q_vector[i]->napi));

	/* Clear any pending interrupts. */
	//读取寄存器E1000_TSICR、E1000_ICR并清除所有挂起的中断
	rd32(E1000_TSICR);
	rd32(E1000_ICR);
	//启用适配器中断
	igb_irq_enable(adapter);
	//是否有分配的VFS功能
	/* notify VFs that reset has been completed */
	if (adapter->vfs_allocated_count) {
		u32 reg_data = rd32(E1000_CTRL_EXT);

		reg_data |= E1000_CTRL_EXT_PFRSTD;//前32位位或操作
		wr32(E1000_CTRL_EXT, reg_data);
	}
	//启动所有的传输队列
	netif_tx_start_all_queues(netdev);
	//如果不是休眠状态，电源管理计数减1
	if (!resuming)
		pm_runtime_put(&pdev->dev);

	/* start the watchdog. */

	hw->mac.get_link_status = 1;
	schedule_work(&adapter->watchdog_task);

	return 0;

err_set_queues:
	igb_free_irq(adapter);
err_req_irq:
	igb_release_hw_control(adapter);
	igb_power_down_link(adapter);
	igb_free_all_rx_resources(adapter);
err_setup_rx:
	igb_free_all_tx_resources(adapter);
err_setup_tx:
	igb_reset(adapter);
	if (!resuming)
		pm_runtime_put(&pdev->dev);

	return err;
}

/**
 *  igb_setup_all_tx_resources - wrapper to allocate Tx resources
 *				 (Descriptors) for all queues
 *  @adapter: board private structure
 *
 *  Return 0 on success, negative on failure
 **/
//为传输队列分配资源
static int igb_setup_all_tx_resources(struct igb_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;
	int i, err = 0;
	//循环遍历所有的传输队列
	for (i = 0; i < adapter->num_tx_queues; i++) {
		err = igb_setup_tx_resources(adapter->tx_ring[i]);
		if (err) {
			//记录错误日志，分配失败的传输队列
			dev_err(&pdev->dev,
				"Allocation for Tx Queue %u failed\n", i);
			//释放已
			for (i--; i >= 0; i--)
				igb_free_tx_resources(adapter->tx_ring[i]);
			break;
		}
	}

	return err;
}

/**
 *  igb_setup_tx_resources - allocate Tx resources (Descriptors)
 *  @tx_ring: tx descriptor ring (for a specific queue) to setup
 *
 *  Return 0 on success, negative on failure
 **/
int igb_setup_tx_resources(struct igb_ring *tx_ring)
{
	struct device *dev = tx_ring->dev;
	int size;
	//申请igb_tx_buffer数组内存，其中tx_ring->count描述了发送环形缓冲区可以同时处理的最大数据包数量
	size = sizeof(struct igb_tx_buffer) * tx_ring->count;

	tx_ring->tx_buffer_info = vmalloc(size);
	//分配失败
	if (!tx_ring->tx_buffer_info)
		goto err;

	/* round up to nearest 4K */
	tx_ring->size = tx_ring->count * sizeof(union e1000_adv_tx_desc);
	//大小按照4KB对齐
	tx_ring->size = ALIGN(tx_ring->size, 4096);
	//dma_alloc_coherent分配了DMA区域，tx_ring->size分配的内存大小、tx_ring->dma分配的内存物理地址、GFP_KERNEL内核上下文进行会产生睡眠
	tx_ring->desc = dma_alloc_coherent(dev, tx_ring->size,
					   &tx_ring->dma, GFP_KERNEL);
	if (!tx_ring->desc)
		goto err;

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	return 0;

err:
	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Tx descriptor ring\n");
	return -ENOMEM;
}
