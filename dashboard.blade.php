<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Dashboard</title>
    <link href="{{ asset('images/login/logo.png') }}" rel="icon">
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet">
    <!-- Include Tailwind CSS -->
    <link href="https://cdn.jsdelivr.net/npm/tailwindcss@2.2.19/dist/tailwind.min.css" rel="stylesheet">
    <link href="https://cdn.jsdelivr.net/npm/flowbite@2.5.2/dist/flowbite.min.css" rel="stylesheet" />
    <meta name="csrf-token" content="{{ csrf_token() }}">
    @vite('resources/css/app.css')

    <style>
        .progress-container {
            position: relative;
            width: 100%;
            height: 20px;
            background: linear-gradient(to right, red, yellow, green);
            border-radius: 10px;
            margin-bottom: 10px;
        }

        .progress-bar-fill {
            height: 100%;
            width: 0;
            background-color: rgba(255, 255, 255, 0.3);
            border-radius: 10px;
        }

        .progress-needle {
            position: absolute;
            top: -10px;
            width: 2px;
            height: 40px;
            background: black;
            transition: left 0.5s;
        }
    </style>

</head>

<body class="flex flex-col h-screen">
    <div class="flex h-screen">
        @livewire('partials.sidebar')
        <!-- Content -->
        <div class="flex flex-col flex-1 overflow-hidden">
            <!-- Header -->
            @livewire('partials.header')

            <!-- Main Content -->
            <main class="flex-1 p-4 overflow-y-auto">
                {{-- Ini untuk alert --}}
                @if (session('success'))
                    <div id="success-alert"
                        class="p-4 mb-4 text-green-800 border border-green-300 rounded-lg bg-green-50 dark:bg-gray-800 dark:text-green-400 dark:border-green-800"
                        role="alert">
                        <span class="font-medium">Success!</span> {{ session('success') }}
                    </div>
                    <script>
                        setTimeout(function() {
                            var alert = document.getElementById('success-alert');
                            if (alert) {
                                alert.style.display = 'none';
                            }
                        }, 5000);
                    </script>
                @endif
                <div class="gap-4">
                    <div class="mb-4 bg-white rounded-lg shadow-lg">
                        <div class="flex items-center justify-between px-4 py-3 border-b border-gray-200 bg-gray-50">
                            <h6 class="m-0 font-semibold text-gray-700">Dashboard</h6>
                            <button type="button"
                                class="ms-auto text-white bg-blue-950 hover:bg-blue-700 focus:ring-4 focus:outline-none font-medium rounded-lg text-sm px-5 py-2.5 text-center inline-flex items-center me-2 dark:bg-blue-600 dark:hover:bg-blue-700 dark:focus:ring-blue-800"
                                data-modal-target="crud-modal" data-modal-toggle="crud-modal">
                                <svg class="w-5 h-5 me-1 -ms-1" fill="currentColor" viewBox="0 0 20 20"
                                    xmlns="http://www.w3.org/2000/svg">
                                    <path fill-rule="evenodd"
                                        d="M10 5a1 1 0 011 1v3h3a1 1 0 110 2h-3v3a1 1 0 11-2 0v-3H6a1 1 0 110-2h3V6a1 1 0 011-1z"
                                        clip-rule="evenodd"></path>
                                </svg>
                                Tambah
                            </button>
                        </div>
                        <div class="grid grid-cols-3 gap-4">
                            <div class="col-span-1">
                                <div class="py-4">
                                    <div
                                        class="flex flex-col items-center justify-center h-full px-8 rounded-lg shadow-lg bg-blue-950">
                                        <div class="flex items-center justify-between w-full my-6">
                                            <p class="text-sm font-bold text-white">Temperature (°C)</p>
                                            <div class="flex items-center">
                                                <div id="colorIndicatorTemperature" class="w-3 h-3 mr-2 rounded-full">
                                                </div>
                                                <p class="text-sm text-white" id="percentageValueTemperature"></p>
                                            </div>
                                        </div>
                                        <p id="temperature" class="pb-2 text-5xl font-extrabold text-white">-</p>
                                        <p class="text-sm text-white">Temperature (°C)</p>
                                        <p class="text-sm text-white">Saat ini</p>
                                        <div id="latestValueTemperature" class="mb-4 text-5xl font-bold text-white">
                                        </div>
                                        <p id="lastUpdatedTemperature" class="text-sm text-white"></p>
                                    </div>
                                </div>
                            </div>
                            <div class="col-span-1">
                                <div class="py-4">
                                    <div class="flex flex-col items-center justify-center h-full px-8 rounded-lg shadow-lg"
                                        style="background-color: #FCA311">
                                        <div class="flex items-center justify-between w-full my-6">
                                            <p class="text-sm font-bold text-white">Humidity (%)</p>
                                            <div class="flex items-center">
                                                <div id="colorIndicatorTemperature" class="w-3 h-3 mr-2 rounded-full">
                                                </div>
                                                <p class="text-sm text-white" id="percentageValueTemperature"></p>
                                            </div>
                                        </div>
                                        <p id="humidity"class="pb-2 text-5xl font-extrabold text-white">-</p>
                                        <p class="text-sm text-white">Humadity (%)</p>
                                        <p class="text-sm text-white">Saat ini</p>
                                        <div id="latestValueTemperature" class="mb-4 text-5xl font-bold text-white">
                                        </div>
                                        <p id="lastUpdatedTemperature" class="text-sm text-white"></p>
                                    </div>
                                </div>
                            </div>
                        </div>

                        <div class="px-8 py-8">
                            <div class="grid grid-cols-1 gap-4">
                                <div class="w-full p-4 bg-white rounded-lg shadow dark:bg-gray-800 md:p-6">
                                    <div class="flex justify-between mb-5">
                                        <div class="grid grid-cols-2 gap-4">
                                            <div>
                                                <h5
                                                    class="inline-flex items-center mb-2 font-normal leading-none text-gray-500 dark:text-gray-400 ">
                                                    HTTP
                                                    <svg data-popover-target="clicks-info"
                                                        data-popover-placement="bottom"
                                                        class="w-3 h-3 text-gray-400 cursor-pointer hover:text-gray-900 dark:hover:text-white ms-1"
                                                        aria-hidden="true" xmlns="http://www.w3.org/2000/svg"
                                                        fill="currentColor" viewBox="0 0 20 20">
                                                        <path
                                                            d="M10 .5a9.5 9.5 0 1 0 9.5 9.5A9.51 9.51 0 0 0 10 .5ZM9.5 4a1.5 1.5 0 1 1 0 3 1.5 1.5 0 0 1 0-3ZM12 15H8a1 1 0 0 1 0-2h1v-3H8a1 1 0 0 1 0-2h2a1 1 0 0 1 1 1v4h1a1 1 0 0 1 0 2Z" />
                                                    </svg>
                                                    <div data-popover id="clicks-info" role="tooltip"
                                                        class="absolute z-10 invisible inline-block text-sm text-gray-500 transition-opacity duration-300 bg-white border border-gray-200 rounded-lg shadow-sm opacity-0 w-72 dark:bg-gray-800 dark:border-gray-600 dark:text-gray-400">
                                                        <div class="p-3 space-y-2">
                                                            <h3 class="font-semibold text-gray-900 dark:text-white">
                                                                HTTP
                                                                growth - Incremental</h3>
                                                            <p>Report helps navigate cumulative growth of community
                                                                activities. Ideally, the chart should have a growing
                                                                trend,
                                                                as stagnating chart signifies a significant decrease of
                                                                community activity.</p>
                                                            <h3 class="font-semibold text-gray-900 dark:text-white">
                                                                Calculation</h3>
                                                            <p>For each date bucket, the all-time volume of activities
                                                                is
                                                                calculated. This means that activities in period n
                                                                contain
                                                                all activities up to period n, plus the activities
                                                                generated
                                                                by your community in period.</p>
                                                            <a href="#"
                                                                class="flex items-center font-medium text-blue-600 dark:text-blue-500 dark:hover:text-blue-600 hover:text-blue-700 hover:underline">Read
                                                                more <svg class="w-2 h-2 ms-1.5 rtl:rotate-180"
                                                                    aria-hidden="true"
                                                                    xmlns="http://www.w3.org/2000/svg" fill="none"
                                                                    viewBox="0 0 6 10">
                                                                    <path stroke="currentColor" stroke-linecap="round"
                                                                        stroke-linejoin="round" stroke-width="2"
                                                                        d="m1 9 4-4-4-4" />
                                                                </svg></a>
                                                        </div>
                                                        <div data-popper-arrow></div>
                                                    </div>
                                                </h5>
                                                <p
                                                    class="text-base font-semibold leading-none text-gray-900 dark:text-white">
                                                    1000 bps</p>
                                            </div>
                                            <div>
                                                <h5
                                                    class="inline-flex items-center mb-2 font-normal leading-none text-gray-500 dark:text-gray-400">
                                                    MP QUIC
                                                    <svg data-popover-target="cpc-info"
                                                        data-popover-placement="bottom"
                                                        class="w-3 h-3 text-gray-400 cursor-pointer hover:text-gray-900 dark:hover:text-white ms-1"
                                                        aria-hidden="true" xmlns="http://www.w3.org/2000/svg"
                                                        fill="currentColor" viewBox="0 0 20 20">
                                                        <path
                                                            d="M10 .5a9.5 9.5 0 1 0 9.5 9.5A9.51 9.51 0 0 0 10 .5ZM9.5 4a1.5 1.5 0 1 1 0 3 1.5 1.5 0 0 1 0-3ZM12 15H8a1 1 0 0 1 0-2h1v-3H8a1 1 0 0 1 0-2h2a1 1 0 0 1 1 1v4h1a1 1 0 0 1 0 2Z" />
                                                    </svg>
                                                    <div data-popover id="cpc-info" role="tooltip"
                                                        class="absolute z-10 invisible inline-block text-sm text-gray-500 transition-opacity duration-300 bg-white border border-gray-200 rounded-lg shadow-sm opacity-0 w-72 dark:bg-gray-800 dark:border-gray-600 dark:text-gray-400">
                                                        <div class="p-3 space-y-2">
                                                            <h3 class="font-semibold text-gray-900 dark:text-white">
                                                                MP QUIC
                                                                growth - Incremental</h3>
                                                            <p>Report helps navigate cumulative growth of community
                                                                activities. Ideally, the chart should have a growing
                                                                trend,
                                                                as stagnating chart signifies a significant decrease of
                                                                community activity.</p>
                                                            <h3 class="font-semibold text-gray-900 dark:text-white">
                                                                Calculation</h3>
                                                            <p>For each date bucket, the all-time volume of activities
                                                                is
                                                                calculated. This means that activities in period n
                                                                contain
                                                                all activities up to period n, plus the activities
                                                                generated
                                                                by your community in period.</p>
                                                            <a href="#"
                                                                class="flex items-center font-medium text-blue-600 dark:text-blue-500 dark:hover:text-blue-600 hover:text-blue-700 hover:underline">Read
                                                                more <svg class="w-2 h-2 ms-1.5 rtl:rotate-180"
                                                                    aria-hidden="true"
                                                                    xmlns="http://www.w3.org/2000/svg" fill="none"
                                                                    viewBox="0 0 6 10">
                                                                    <path stroke="currentColor" stroke-linecap="round"
                                                                        stroke-linejoin="round" stroke-width="2"
                                                                        d="m1 9 4-4-4-4" />
                                                                </svg></a>
                                                        </div>
                                                        <div data-popper-arrow></div>
                                                    </div>
                                                </h5>
                                                <p
                                                    class="text-base font-semibold leading-none text-gray-900 dark:text-white">
                                                    1000000 bps</p>
                                            </div>
                                        </div>
                                        <div>
                                            <button id="dropdownDefaultButton" data-dropdown-toggle="lastDaysdropdown"
                                                data-dropdown-placement="bottom" type="button"
                                                class="inline-flex items-center px-3 py-2 text-sm font-medium text-gray-900 bg-white border border-gray-200 rounded-lg focus:outline-none hover:bg-gray-100 hover:text-blue-700 focus:z-10 focus:ring-4 focus:ring-gray-200 dark:focus:ring-gray-700 dark:bg-gray-800 dark:text-gray-400 dark:border-gray-600 dark:hover:text-white dark:hover:bg-gray-700">Last
                                                week <svg class="w-2.5 h-2.5 ms-2.5" aria-hidden="true"
                                                    xmlns="http://www.w3.org/2000/svg" fill="none"
                                                    viewBox="0 0 10 6">
                                                    <path stroke="currentColor" stroke-linecap="round"
                                                        stroke-linejoin="round" stroke-width="2" d="m1 1 4 4 4-4" />
                                                </svg></button>
                                            <div id="lastDaysdropdown"
                                                class="z-10 hidden bg-white divide-y divide-gray-100 rounded-lg shadow w-44 dark:bg-gray-700">
                                                <ul class="py-2 text-sm text-gray-700 dark:text-gray-200"
                                                    aria-labelledby="dropdownDefaultButton">
                                                    <li>
                                                        <a href="#"
                                                            class="block px-4 py-2 hover:bg-gray-100 dark:hover:bg-gray-600 dark:hover:text-white">Yesterday</a>
                                                    </li>
                                                    <li>
                                                        <a href="#"
                                                            class="block px-4 py-2 hover:bg-gray-100 dark:hover:bg-gray-600 dark:hover:text-white">Today</a>
                                                    </li>
                                                    <li>
                                                        <a href="#"
                                                            class="block px-4 py-2 hover:bg-gray-100 dark:hover:bg-gray-600 dark:hover:text-white">Last
                                                            7 days</a>
                                                    </li>
                                                    <li>
                                                        <a href="#"
                                                            class="block px-4 py-2 hover:bg-gray-100 dark:hover:bg-gray-600 dark:hover:text-white">Last
                                                            30 days</a>
                                                    </li>
                                                    <li>
                                                        <a href="#"
                                                            class="block px-4 py-2 hover:bg-gray-100 dark:hover:bg-gray-600 dark:hover:text-white">Last
                                                            90 days</a>
                                                    </li>
                                                </ul>
                                            </div>
                                        </div>
                                    </div>
                                    <div id="line-chart"></div>
                                    {{-- <div
                                        class="grid grid-cols-1 items-center border-gray-200 border-t dark:border-gray-700 justify-between mt-2.5">
                                        <div class="pt-5">
                                            <a href="#"
                                                class="px-5 py-2.5 text-sm font-medium text-white inline-flex items-center bg-blue-700 hover:bg-blue-800 focus:ring-4 focus:outline-none focus:ring-blue-300 rounded-lg text-center dark:bg-blue-600 dark:hover:bg-blue-700 dark:focus:ring-blue-800">
                                                <svg class="w-3.5 h-3.5 text-white me-2 rtl:rotate-180"
                                                    aria-hidden="true" xmlns="http://www.w3.org/2000/svg"
                                                    fill="currentColor" viewBox="0 0 16 20">
                                                    <path
                                                        d="M14.066 0H7v5a2 2 0 0 1-2 2H0v11a1.97 1.97 0 0 0 1.934 2h12.132A1.97 1.97 0 0 0 16 18V2a1.97 1.97 0 0 0-1.934-2Zm-3 15H4.828a1 1 0 0 1 0-2h6.238a1 1 0 0 1 0 2Zm0-4H4.828a1 1 0 0 1 0-2h6.238a1 1 0 1 1 0 2Z" />
                                                    <path
                                                        d="M5 5V.13a2.96 2.96 0 0 0-1.293.749L.879 3.707A2.98 2.98 0 0 0 .13 5H5Z" />
                                                </svg>
                                                View full report
                                            </a>
                                        </div>
                                    </div> --}}
                                </div>
                            </div>
                        </div>
                        <div class="gap-4">
                            <div class="mb-4 bg-white rounded-lg shadow-lg">
                                <div class="px-8 py-8">
                                    <div class="relative overflow-x-auto shadow-md sm:rounded-lg">
                                        {{-- tabel --}}
                                        <table
                                            class="w-full text-sm text-left text-gray-500 rtl:text-right dark:text-gray-400">
                                            <thead
                                                class="text-xs text-white uppercase dark:bg-gray-700 dark:text-gray-400"
                                                style="background-color: #FCA311">
                                                <tr>
                                                    <th scope="col" class="px-6 py-3">ID</th>
                                                    <th scope="col" class="px-6 py-3">Jenis Protokol</th>
                                                    <th scope="col" class="px-6 py-3">Microcontroller</th>
                                                    <th scope="col" class="px-6 py-3">MAC Address</th>
                                                    <th scope="col" class="px-6 py-3">IP Address</th>
                                                    <th scope="col" class="px-6 py-3">Throughput</th>
                                                    <th scope="col" class="px-6 py-3">Latency</th>
                                                </tr>
                                            </thead>
                                            <tbody>
                                                @foreach ($alat as $item)
                                                    <tr
                                                        class="border-b odd:bg-white odd:dark:bg-gray-900 even:bg-gray-50 even:dark:bg-gray-800 dark:border-gray-700">
                                                        <th scope="row"
                                                            class="px-6 py-4 font-medium text-gray-900 whitespace-nowrap dark:text-white">
                                                            {{ $item->id_alat }}
                                                        </th>
                                                        <td class="px-6 py-4">{{ $item->protocol }}</td>
                                                        <td class="px-6 py-4">{{ $item->microcontroller }}</td>
                                                        <td class="px-6 py-4">{{ $item->mac_address }}</td>
                                                        <td class="px-6 py-4">{{ $item->ip_address }}</td>
                                                        <td class="px-6 py-4">
                                                            @foreach ($item->data as $data)
                                                                <div>{{ $data->throughput }}</div>
                                                            @endforeach
                                                        </td>
                                                        <td class="px-6 py-4">
                                                            @foreach ($item->data as $data)
                                                                <div>{{ $data->latency }}</div>
                                                            @endforeach
                                                        </td>
                                                    </tr>
                                                @endforeach
                                            </tbody>
                                        </table>



                                        {{-- <nav class="flex flex-wrap items-center justify-between pt-4 pb-4 ml-8 mr-5 flex-column md:flex-row"
                                            aria-label="Table navigation">
                                            <span
                                                class="block w-full mb-4 text-sm font-normal text-gray-500 dark:text-gray-400 md:mb-0 md:inline md:w-auto">Showing
                                                <span class="font-semibold text-gray-900 dark:text-white">1-10</span> of
                                                <span class="font-semibold text-gray-900 dark:text-white">1000</span></span>
                                            <ul class="inline-flex h-8 -space-x-px text-sm rtl:space-x-reverse">
                                                <li>
                                                    <a href="#"
                                                        class="flex items-center justify-center h-8 px-3 leading-tight text-gray-500 bg-white border border-gray-300 ms-0 rounded-s-lg hover:bg-gray-100 hover:text-gray-700 dark:bg-gray-800 dark:border-gray-700 dark:text-gray-400 dark:hover:bg-gray-700 dark:hover:text-white">Previous</a>
                                                </li>
                                                <li>
                                                    <a href="#"
                                                        class="flex items-center justify-center h-8 px-3 leading-tight text-gray-500 bg-white border border-gray-300 hover:bg-gray-100 hover:text-gray-700 dark:bg-gray-800 dark:border-gray-700 dark:text-gray-400 dark:hover:bg-gray-700 dark:hover:text-white">1</a>
                                                </li>
                                                <li>
                                                    <a href="#"
                                                        class="flex items-center justify-center h-8 px-3 leading-tight text-gray-500 bg-white border border-gray-300 hover:bg-gray-100 hover:text-gray-700 dark:bg-gray-800 dark:border-gray-700 dark:text-gray-400 dark:hover:bg-gray-700 dark:hover:text-white">2</a>
                                                </li>
                                                <li>
                                                    <a href="#" aria-current="page"
                                                        class="flex items-center justify-center h-8 px-3 text-blue-600 border border-gray-300 bg-blue-50 hover:bg-blue-100 hover:text-blue-700 dark:border-gray-700 dark:bg-gray-700 dark:text-white">3</a>
                                                </li>
                                                <li>
                                                    <a href="#"
                                                        class="flex items-center justify-center h-8 px-3 leading-tight text-gray-500 bg-white border border-gray-300 hover:bg-gray-100 hover:text-gray-700 dark:bg-gray-800 dark:border-gray-700 dark:text-gray-400 dark:hover:bg-gray-700 dark:hover:text-white">4</a>
                                                </li>
                                                <li>
                                                    <a href="#"
                                                        class="flex items-center justify-center h-8 px-3 leading-tight text-gray-500 bg-white border border-gray-300 hover:bg-gray-100 hover:text-gray-700 dark:bg-gray-800 dark:border-gray-700 dark:text-gray-400 dark:hover:bg-gray-700 dark:hover:text-white">5</a>
                                                </li>
                                                <li>
                                                    <a href="#"
                                                        class="flex items-center justify-center h-8 px-3 leading-tight text-gray-500 bg-white border border-gray-300 rounded-e-lg hover:bg-gray-100 hover:text-gray-700 dark:bg-gray-800 dark:border-gray-700 dark:text-gray-400 dark:hover:bg-gray-700 dark:hover:text-white">Next</a>
                                                </li>
                                            </ul>
                                        </nav> --}}

                                        {{-- <div id="crud-modal" tabindex="-1" aria-hidden="true"
                                            class="hidden overflow-y-auto overflow-x-hidden fixed top-0 right-0 left-0 z-50 justify-center items-center w-full md:inset-0 h-[calc(100%-1rem)] max-h-full">
                                            <div class="relative w-full max-w-md max-h-full p-4">
                                                <!-- Modal content -->
                                                <div class="relative bg-white rounded-lg shadow dark:bg-gray-700">
                                                    <!-- Modal header -->
                                                    <div
                                                        class="flex items-center justify-between p-4 border-b rounded-t md:p-5 dark:border-gray-600">
                                                        <h3
                                                            class="text-lg font-semibold text-gray-900 dark:text-white">
                                                            Tambah Microcontroller
                                                        </h3>
                                                        <button type="button"
                                                            class="inline-flex items-center justify-center w-8 h-8 text-sm text-gray-400 bg-transparent rounded-lg hover:bg-gray-200 hover:text-gray-900 ms-auto dark:hover:bg-gray-600 dark:hover:text-white"
                                                            data-modal-toggle="crud-modal">
                                                            <svg class="w-3 h-3" aria-hidden="true"
                                                                xmlns="http://www.w3.org/2000/svg" fill="none"
                                                                viewBox="0 0 14 14">
                                                                <path stroke="currentColor" stroke-linecap="round"
                                                                    stroke-linejoin="round" stroke-width="2"
                                                                    d="m1 1 6 6m0 0 6 6M7 7l6-6M7 7l-6 6" />
                                                            </svg>
                                                            <span class="sr-only">Close modal</span>
                                                        </button>
                                                    </div>
                                                    <!-- Modal body -->
                                                    <form class="p-4 md:p-5">
                                                        <div class="grid grid-cols-2 gap-4 mb-4">
                                                            <div class="col-span-2">
                                                                <label for="idmicrocontroller"
                                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">Id
                                                                    Microcontroller</label>
                                                                <input type="text" name="idmicrocontroller"
                                                                    id="idmicrocontroller"
                                                                    class="mb-2 bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-primary-600 focus:border-primary-600 block w-full p-2.5 dark:bg-gray-600 dark:border-gray-500 dark:placeholder-gray-400 dark:text-white dark:focus:ring-primary-500 dark:focus:border-primary-500"
                                                                    placeholder="Microcontroller" required=""
                                                                    value="106" readonly>
                                                                <label for="name"
                                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">Jenis
                                                                    Microcontroller</label>
                                                                <select id="name"
                                                                    class="mb-2 bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-primary-500 focus:border-primary-500 block w-full p-2.5 dark:bg-gray-600 dark:border-gray-500 dark:placeholder-gray-400 dark:text-white dark:focus:ring-primary-500 dark:focus:border-primary-500">
                                                                    <option selected="microcontroller">Pilih
                                                                        Microcontroller
                                                                    </option>
                                                                    <option value="esp32">ESP32</option>
                                                                    </option>
                                                                    <option value="esp8266">ESP8266</option>
                                                                    <option value="Rpi">Raspberry Pi</option>
                                                                </select>
                                                                <label for="macaddress"
                                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">
                                                                    Mac Address</label>
                                                                <input type="text" name="macaddress"
                                                                    id="macaddress"
                                                                    class="mb-2 bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-primary-600 focus:border-primary-600 block w-full p-2.5 dark:bg-gray-600 dark:border-gray-500 dark:placeholder-gray-400 dark:text-white dark:focus:ring-primary-500 dark:focus:border-primary-500"
                                                                    placeholder="contoh: 00-B0-D0-63-C2-26"
                                                                    required="">
                                                                <label for="ipaddress"
                                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">
                                                                    IP Address</label>
                                                                <input type="text" name="ipaddress" id="ipaddress"
                                                                    class="mb-2 bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-primary-600 focus:border-primary-600 block w-full p-2.5 dark:bg-gray-600 dark:border-gray-500 dark:placeholder-gray-400 dark:text-white dark:focus:ring-primary-500 dark:focus:border-primary-500"
                                                                    placeholder="contoh: 192.164.001.001"
                                                                    required="">
                                                            </div>
                                                        </div>
                                                        <button type="submit"
                                                            class="text-white inline-flex items-center bg-blue-950 hover:bg-blue-700 focus:ring-4 focus:outline-none focus:ring-blue-300 font-medium rounded-lg text-sm px-5 py-2.5 text-center dark:bg-blue-600 dark:hover:bg-blue-700 dark:focus:ring-blue-800">
                                                            Simpan
                                                        </button>
                                                    </form>
                                                </div>
                                            </div>
                                        </div> --}}
                                    </div>
                                </div>
                            </div>
                        </div>
                        {{-- Modal --}}
                        <div id="crud-modal" tabindex="-1"
                            class="fixed top-0 left-0 right-0 z-50 hidden w-full p-4 overflow-x-hidden overflow-y-auto md:inset-0 h-modal md:h-full">
                            <div class="relative w-full h-full max-w-md md:h-auto">
                                <div class="relative bg-white rounded-lg shadow dark:bg-gray-700">
                                    <button type="button"
                                        class="absolute top-3 right-2.5 text-gray-400 bg-transparent hover:bg-gray-200 hover:text-gray-900 rounded-lg text-sm p-1.5 ml-auto inline-flex items-center dark:hover:bg-gray-800 dark:hover:text-white"
                                        data-modal-toggle="crud-modal">
                                        <svg aria-hidden="true" class="w-5 h-5" fill="currentColor"
                                            viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">
                                            <path fill-rule="evenodd"
                                                d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z"
                                                clip-rule="evenodd"></path>
                                        </svg>
                                        <span class="sr-only">Close modal</span>
                                    </button>
                                    <div class="px-6 py-6 lg:px-8">
                                        <h3 class="mb-4 text-xl font-medium text-gray-900 dark:text-white">Tambah Alat
                                        </h3>
                                        <form class="space-y-6" action="/dashboard" method="POST">
                                            @csrf
                                            <div>
                                                <label for="protocol"
                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">Protocol</label>
                                                <select id="protocol" name="protocol"
                                                    class="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-blue-500 focus:border-blue-500 block w-full p-2.5">
                                                    <option selected="protocol">Pilih Protocol
                                                    </option>
                                                    <option value="http">HTTP</option>
                                                    <option value="mpquic">MPQUIC</option>
                                                </select>
                                            </div>
                                            <div>
                                                <label for="microcontroller"
                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">Microcontroller</label>
                                                <select id="microcontroller" name="microcontroller"
                                                    class="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-blue-500 focus:border-blue-500 block w-full p-2.5">
                                                    <option selected="microcontroller">Pilih Microcontroller
                                                    </option>
                                                    <option value="esp32">ESP32</option>
                                                    <option value="esp8266">ESP8266</option>
                                                    <option value="raspberrypi">Raspberry Pi</option>
                                                </select>
                                            </div>
                                            <div>
                                                <label for="mac_address"
                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">MAC
                                                    Address</label>
                                                <input type="text" name="mac_address" id="mac_address"
                                                    maxlength="20"
                                                    class="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-blue-500 focus:border-blue-500 block w-full p-2.5"
                                                    placeholder="contoh: 00-B0-D0-63-C2-26" required>
                                            </div>
                                            <div>
                                                <label for="ip_address"
                                                    class="block mb-2 text-sm font-medium text-gray-900 dark:text-white">IP
                                                    Address</label>
                                                <input type="text" name="ip_address" id="ip_address"
                                                    maxlength="16"
                                                    class="bg-gray-50 border border-gray-300 text-gray-900 text-sm rounded-lg focus:ring-blue-500 focus:border-blue-500 block w-full p-2.5"
                                                    placeholder="contoh: 192.164.001.001" required>
                                            </div>
                                            <button type="submit"
                                                class="w-full text-white bg-blue-950 hover:bg-blue-700 focus:ring-4 focus:outline-none font-medium rounded-lg text-sm px-5 py-2.5 text-center dark:bg-blue-600 dark:hover:bg-blue-700 dark:focus:ring-blue-800">Tambah</button>
                                        </form>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </main>

            <!-- Footer -->
            @livewire('partials.footer')

            <!-- Logout form (hidden by default) -->
            <form id="logout-form" action="#" method="POST" style="display: none;">
                @csrf
                <input type="hidden" name="logout" value="true">
            </form>
        </div>
    </div>
    @vite('resources/js/app.js')

    <script src="https://cdn.jsdelivr.net/npm/flowbite@2.5.2/dist/flowbite.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/apexcharts"></script>
    <script>
        const options1 = {
            chart: {
                height: "100%",
                maxWidth: "100%",
                type: "line",
                fontFamily: "Inter, sans-serif",
                dropShadow: {
                    enabled: false,
                },
                toolbar: {
                    show: false,
                },
            },
            tooltip: {
                enabled: true,
                x: {
                    show: false,
                },
            },
            dataLabels: {
                enabled: false,
            },
            stroke: {
                width: 6,
            },
            grid: {
                show: true,
                strokeDashArray: 4,
                padding: {
                    left: 2,
                    right: 2,
                    top: -26
                },
            },
            series: [{
                    name: "HTTP",
                    data: [6500, 6418, 6456, 6526, 6356, 6456],
                    color: "#001D3D",
                },
                {
                    name: "MP QUIC",
                    data: [6456, 6356, 6526, 6332, 6418, 6500],
                    color: "#FCA311",
                },
            ],
            legend: {
                show: false
            },
            stroke: {
                curve: 'smooth'
            },
            xaxis: {
                categories: ['01 Feb', '02 Feb', '03 Feb', '04 Feb', '05 Feb', '06 Feb', '07 Feb'],
                labels: {
                    show: true,
                    style: {
                        fontFamily: "Inter, sans-serif",
                        cssClass: 'text-xs font-normal fill-gray-500 dark:fill-gray-400'
                    }
                },
                axisBorder: {
                    show: false,
                },
                axisTicks: {
                    show: false,
                },
            },
            yaxis: {
                show: false,
            },
        }

        if (document.getElementById("line-chart") && typeof ApexCharts !== 'undefined') {
            const chart1 = new ApexCharts(document.getElementById("line-chart"), options1);
            chart1.render();
        };
    </script>

    {{-- <script>
        // Membuat koneksi SSE ke endpoint /data
        const eventSource = new EventSource('/data');

        eventSource.onmessage = function(event) {
            const data = JSON.parse(event.data);
            document.getElementById('temperature').textContent = data.temperature ?? '-';
            document.getElementById('humidity').textContent = data.humidity ?? '-';
        };
    </script> --}}
    <script>
        const socket = new WebSocket('ws://localhost:3000');

        socket.onmessage = function(event) {
            const data = JSON.parse(event.data);
            document.getElementById('temperature').textContent = data.temperature;
            document.getElementById('humidity').textContent = data.humidity;
        };

        socket.onopen = function() {
            console.log('Connected to WebSocket server');
        };
    </script>
</body>

</html>
