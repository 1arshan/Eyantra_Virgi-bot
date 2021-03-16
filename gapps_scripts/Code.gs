function doGet(e){                
  const debg = false                               
  const shouldSentEmail = true       
                                                  
  const parameter = debg? {"id":"IncomingOrders","Order ID":"1002","city":"Mumbai"        ,"Shipped Quantity":"1" ,"Cost":150,"Team Id":'VB#1637',"Unique Id":"VBfetjmi","Item":"Clothes","Priority":"LP",'Shipped Status':"Yes",'Shipped Date and Time':'Sat Feb 20 2021 - 16:30:15',"deliveryIn":"5","Estimated Time of Delivery":"Sun Feb 21 2021 - 16:30:15"}: e.parameter;
  const ss = SpreadsheetApp.getActive();
  const sheet = ss.getSheetByName(parameter["id"]);
  const headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  const lastRow = sheet.getLastRow();
    
  let cell = sheet.getRange('a1');
  let col = 0;

  let d = new Date();
  
  const timeStamp =  d.toDateString() + ", " + d.toLocaleTimeString()
  
  const addDays=(date, days)=>{
  var result = new Date(date);
  result.setDate(result.getDate() + days);
  return result;
  }

  const get_order_summary=(obj)=>{
    let str = ''
    Object.keys(obj).map(key=>{
      str =str + key+' : ' + obj[key].toString() + '\n'
    })
    console.log(str)
    return str;
  }
  
  const UpdateDashboard=(id,column,value)=>{
    const dashSheet = ss.getSheetByName('Dashboard');
    const dashCell = dashSheet.getRange('a1');
    for(let i = dashSheet.getLastRow(); i>=2 ;i--){
    let orderId = dashSheet.getRange(i, 1).getValue();
    if(orderId.toString() === id.toString()){
        dashCell.offset(i-1, column).setValue(value);
      }  
    }
  }

const getValueDashboard=(sh,id,column,orderIdIndex)=>{
    const dashSheet = ss.getSheetByName(sh);
    for(let i = dashSheet.getLastRow(); i>=2 ;i--){
    let orderId = dashSheet.getRange(i, orderIdIndex).getValue();
    console.log(orderId,'Ggg')
    if(orderId.toString() === id.toString()){
        return dashSheet.getRange(i, column).getValue();
      }  
    }
  }

if(parameter.id === 'IncomingOrders'){
    let column = 0
    const dashSheet = ss.getSheetByName('Dashboard');
    const dashCell = dashSheet.getRange('a1');
    const dashHeaders = dashSheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
    let row = dashSheet.getLastRow();
    for (i in dashHeaders){
    // loop through the headers and if a parameter name matches the header name insert the value
      val = parameter[dashHeaders[i]]
    // append data to the last row
    dashCell.offset(row, column).setValue(val);
    column++;
    }
    UpdateDashboard(parameter['Order ID'],5,'No')
    UpdateDashboard(parameter['Order ID'],6,'No')
    UpdateDashboard(parameter['Order ID'],9,parameter['Order Date and Time']);  
  }
if(parameter.id === 'OrdersDispatched'){
    UpdateDashboard(parameter['Order ID'],5,'Yes')
    UpdateDashboard(parameter['Order ID'],10,parameter['Dispatch Date and Time']);   
  }
if(parameter.id === 'OrdersShipped'){
    UpdateDashboard(parameter['Order ID'],6,'Yes')
    UpdateDashboard(parameter['Order ID'],11,parameter['Shipped Date and Time'])  
    const shippingTime = new Date(getValueDashboard('Dashboard',parameter['Order ID'],12,1).replace(' -',','))
    const incomingTime = new Date(getValueDashboard('Dashboard',parameter['Order ID'],10,1))
    const timeTaken = Math.abs(shippingTime - incomingTime)/1000;
    UpdateDashboard(parameter['Order ID'],12,timeTaken);
  }


for (i in headers){
    // loop through the headers and if a parameter name matches the header name insert the value
    if (headers[i] == "Timestamp")
    {
      val = timeStamp;
    }
   else 
    {
      val = parameter[headers[i]]
    }
    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;
    }
  

    if(parameter["id"] === 'OrdersDispatched' && shouldSentEmail){
      var to = "salimreza0504@gmail.com";   //write your email id here
      var message = "Hello! Your Order has been dispatched, Contact us if you any quetions.We are here to help you.\nOrder Summary :\n" + get_order_summary(parameter) + " \n" ; 
      MailApp.sendEmail(to, "Your Order is Dispatched!", message);
    }

    if(parameter["id"] === 'OrdersShipped' && shouldSentEmail){
      var to = "salimreza0504@gmail.com";   //write your email id here
      const deliveryString = parseInt(parameter.deliveryIn,0)===1?".":"s.";
      var message = "Hello! Your Order has been shipped, It will be drone delivered to you in " + parameter.deliveryIn +"day"+deliveryString+" Contact us if you any quetions. We are here to help you.\nOrder Summary :\n" + get_order_summary(parameter) + " \n" ; 
      MailApp.sendEmail(to, "Your Order is Shipped!", message);
    }
    

  return ContentService.createTextOutput('success');
}

