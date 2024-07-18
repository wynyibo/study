## 网卡启动

网卡均支持多队列，网卡在启动过程时完成的任务就是分配和初始化RingBuffer，在数据包发送中直接在启动网卡的时候分配传输队列。

网卡启动的时候会调用`__igb_open`函数完成RingBuffer分配。

核心操作：

```c
static int __igb_open(struct net_device *netdev, bool resuming)
{
  //分配传输描述符数组，为传输队列分配资源
  /* allocate transmit descriptors */
  err = igb_setup_all_tx_resources(adapter);
 //分配接收描述符数组，为接收队列分配资源
  err = igb_setup_all_rx_resources(adapter);
  //启动所有的传输队列
  netif_tx_start_all_queues(netdev);
}
```

`igb_setup_all_tx_resources`为传输队列分配资源

```c
static int igb_setup_all_tx_resources(struct igb_adapter *adapter)
{
  struct pci_dev *pdev = adapter->pdev;
  int i, err = 0;
  //循环遍历所有的传输队列
  for (i = 0; i < adapter->num_tx_queues; i++) {
​    err = igb_setup_tx_resources(adapter->tx_ring[i]);
​    if (err) {
​      dev_err(&pdev->dev,
​        "Allocation for Tx Queue %u failed\n", i);
​      for (i--; i >= 0; i--)
​        igb_free_tx_resources(adapter->tx_ring[i]);
​      break;
​    }
  }
  return err;
}
```

循环遍历所有的传输队列，igb_setup_tx_resources完成真正的RingBuffer构造，dev_err记录错误日志并分配失败的传输队列，释放已分配的资源，从当前队列向前遍历，逐个释放已分配资源。

```c
int igb_setup_tx_resources(struct igb_ring *tx_ring)
{
  struct device *dev = tx_ring->dev;
  int size;

  size = sizeof(struct igb_tx_buffer) * tx_ring->count;
  tx_ring->tx_buffer_info = vmalloc(size);

  if (!tx_ring->tx_buffer_info)
​    goto err;
  tx_ring->size = tx_ring->count * sizeof(union e1000_adv_tx_desc);

  tx_ring->size = ALIGN(tx_ring->size, 4096);

  tx_ring->desc = dma_alloc_coherent(dev, tx_ring->size,

​            &tx_ring->dma, GFP_KERNEL);

  tx_ring->next_to_use = 0;
  tx_ring->next_to_clean = 0;
  return 0;
}
```

核心分配传输队列其实是分配了两个数组，`igb_tx_buffer` 由`vmalloc`分配供内核使用，`e1000_adv_tx_desc`由`dma_alloc_coherent`分配供网卡硬件使用。

网卡驱动总结：