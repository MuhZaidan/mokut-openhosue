var gateway = `ws://${location.host}/ws`;
const socket = new WebSocket(gateway);

var data = [];
var firstDataStatus = true;
var secondDataStatus = false;
var thirdDataStatus = false;

function sendData(datas) {
  console.log({ datas });
  if (datas == 1) {
    let json = {
      route1: data[0],
      route2: data[1],
      route3: data[2],
    };
    socket.send(JSON.stringify(json));
    return;
  }

  let jsonFail = {
    route1: 'null',
    route2: 'null',
    route3: 'null',
  };
  socket.send(JSON.stringify(jsonFail));
  data.splice(0, data.length);
  updateDataRoute(1);
  return;
}

searchRoute.addEventListener('change', (event) => {
  console.log(event.target.value);
});

searchRoute.addEventListener('click', (event) => {
  displayPopup.classList.add('showed');
  displayPopup.classList.remove('hidden');
});

routeOne.addEventListener('click', (event) => {
  displayPopup.classList.remove('showed');
  displayPopup.classList.add('hidden');
  data.push('Route 1');
  updateDataRoute(0);
});

routeTwo.addEventListener('click', (event) => {
  displayPopup.classList.remove('showed');
  displayPopup.classList.add('hidden');
  data.push('Route 2');
  updateDataRoute(0);
});

routeThree.addEventListener('click', (event) => {
  displayPopup.classList.remove('showed');
  displayPopup.classList.add('hidden');
  data.push('Route 3');
  updateDataRoute(0);
});

const updateDataRoute = (dataX) => {
  if (firstDataStatus == true) {
    route1.classList.add('current-route');
    data.forEach((route) => {
      route1.innerHTML = route;
    });
    firstDataStatus = false;
    secondDataStatus = true;
    return;
  } else if (secondDataStatus == true) {
    route2.classList.add('next-route');
    data.forEach((route) => {
      route2.innerHTML = route;
    });
    secondDataStatus = false;
    thirdDataStatus = true;
    return;
  } else if (thirdDataStatus == true) {
    route3.classList.add('next-route');
    data.forEach((route) => {
      route3.innerHTML = route;
    });
    thirdDataStatus = false;
    firstDataStatus = false;
    return;
  }

  if (dataX == 1) {
    route1.classList.remove('current-route');
    route2.classList.remove('next-route');
    route3.classList.remove('next-route');

    route1.innerHTML = '';
    route2.innerHTML = '';
    route3.innerHTML = '';
    firstDataStatus = true;
    secondDataStatus = false;
    thirdDataStatus = false;
    return;
  }
};
