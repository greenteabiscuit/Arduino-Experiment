const SERVICE_UUID = 'd5875408-fa51-4763-a75d-7d33cecebc31'
const CHARACTERISTIC_UUID = 'a4f01d8c-a037-43b6-9050-1876a8c23584'

const btn = document.getElementById('btn')
const text = document.getElementById('text')

// Web Bluetoothはユーザーアクションをトリガーに処理を始める必要がある
btn.addEventListener('click', (event) => {
  connect()
})

const connect = () => {
  // Scan
  navigator.bluetooth.requestDevice({
    // 'Nefry'というデバイス名でフィルタリング
    acceptAllDevices: false,
    filters: [
      {namePrefix: 'Nefry'}
    ],
    optionalServices: [
      // 使用したいServiceを登録しておく
      SERVICE_UUID
    ]
  })
    // 接続
    .then(device => device.gatt.connect())
    // Service取得
    .then(server => server.getPrimaryService(SERVICE_UUID))
    // Characteristic取得
    .then(service => service.getCharacteristic(CHARACTERISTIC_UUID))
    // Notificationsを開始
    .then(characteristic => setNotifications(characteristic))
    // Errorはこちら
    .catch(error => console.log(error))
}

// Notification設定
const setNotifications = (characteristic) => {

  // Add Event
  characteristic.addEventListener('characteristicvaluechanged', (event) => {
    const value = event.target.value

    // データをパース
    const decoder = new TextDecoder('utf-8')
    const str = decoder.decode(value)
    const json = JSON.parse(str)
    // Nefry BTからのデータを表示
    if (text) text.innerHTML = json.val
  })

  // Notifications開始
  characteristic.startNotifications()
}
